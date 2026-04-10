#!/usr/bin/env python3
from __future__ import annotations

import argparse
import heapq
import json
import math
from collections import defaultdict
from pathlib import Path


def _dist2d(a: list[float], b: list[float]) -> float:
    return math.hypot(b[0] - a[0], b[1] - a[1])


def _turn_deg(a: list[float], b: list[float], c: list[float]) -> float:
    v1x = b[0] - a[0]
    v1y = b[1] - a[1]
    v2x = c[0] - b[0]
    v2y = c[1] - b[1]
    l1 = math.hypot(v1x, v1y)
    l2 = math.hypot(v2x, v2y)
    if l1 < 1e-6 or l2 < 1e-6:
        return 0.0
    dot = (v1x * v2x + v1y * v2y) / (l1 * l2)
    dot = max(-1.0, min(1.0, dot))
    return math.degrees(math.acos(dot))


def _point_line_dist2d(a: list[float], b: list[float], p: list[float]) -> float:
    abx = b[0] - a[0]
    aby = b[1] - a[1]
    apx = p[0] - a[0]
    apy = p[1] - a[1]
    den = abx * abx + aby * aby
    if den < 1e-6:
        return _dist2d(a, p)
    t = (apx * abx + apy * aby) / den
    t = max(0.0, min(1.0, t))
    qx = a[0] + abx * t
    qy = a[1] + aby * t
    return math.hypot(p[0] - qx, p[1] - qy)


def _nearest_node(nodes: list[list[float]], target: tuple[float, float, float]) -> int:
    best_i = 0
    best_d = 1e30
    t = [target[0], target[1], target[2]]
    for i, p in enumerate(nodes):
        d = _dist2d(p, t) + abs(p[2] - t[2]) * 0.5
        if d < best_d:
            best_d = d
            best_i = i
    return best_i


def _build_undirected_graph(
    waypoints: list[dict],
    max_raw_edge_dist2d: float,
    max_raw_edge_abs_dz: float,
) -> dict[int, list[tuple[int, float]]]:
    g: dict[int, list[tuple[int, float]]] = defaultdict(list)
    n = len(waypoints)
    for i, wp in enumerate(waypoints):
        pi = wp["pos"]
        for j in wp.get("next_indices", []):
            if not isinstance(j, int) or j < 0 or j >= n or j == i:
                continue
            pj = waypoints[j]["pos"]
            d2 = _dist2d(pi, pj)
            dz = abs(pj[2] - pi[2])
            if d2 > max_raw_edge_dist2d or dz > max_raw_edge_abs_dz:
                continue
            w = _dist2d(pi, pj) + abs(pj[2] - pi[2]) * 0.3
            g[i].append((j, w))
            g[j].append((i, w))
    return g


def _dijkstra_path(g: dict[int, list[tuple[int, float]]], s: int, t: int) -> list[int]:
    if s == t:
        return [s]
    pq: list[tuple[float, int]] = [(0.0, s)]
    dist = {s: 0.0}
    prev: dict[int, int] = {}
    while pq:
        d, u = heapq.heappop(pq)
        if u == t:
            break
        if d != dist.get(u, 1e30):
            continue
        for v, w in g.get(u, []):
            nd = d + w
            if nd < dist.get(v, 1e30):
                dist[v] = nd
                prev[v] = u
                heapq.heappush(pq, (nd, v))
    if t not in dist:
        return []
    path = [t]
    cur = t
    while cur != s:
        cur = prev[cur]
        path.append(cur)
    path.reverse()
    return path


def _can_shortcut(
    path: list[int],
    nodes: list[list[float]],
    i: int,
    j: int,
    max_direct_dist2d: float,
    max_direct_up_dz: float,
    max_direct_abs_dz: float,
    corridor_width2d: float,
) -> bool:
    if j <= i + 1:
        return True
    a = nodes[path[i]]
    b = nodes[path[j]]
    d2 = _dist2d(a, b)
    if d2 > max_direct_dist2d:
        return False
    up_dz = b[2] - a[2]
    abs_dz = abs(up_dz)
    if up_dz > max_direct_up_dz or abs_dz > max_direct_abs_dz:
        return False
    for k in range(i + 1, j):
        p = nodes[path[k]]
        if _point_line_dist2d(a, b, p) > corridor_width2d:
            return False
    return True


def _compress_path(
    path: list[int],
    nodes: list[list[float]],
    keep_turn_deg: float,
    sample_spacing: float,
    max_direct_dist2d: float,
    max_direct_up_dz: float,
    max_direct_abs_dz: float,
    corridor_width2d: float,
) -> list[int]:
    if len(path) <= 2:
        return path

    must_keep: set[int] = {0, len(path) - 1}
    accum = 0.0
    for i in range(1, len(path) - 1):
        prev_i = i - 1
        a = nodes[path[prev_i]]
        b = nodes[path[i]]
        c = nodes[path[i + 1]]
        accum += _dist2d(a, b)
        turn = _turn_deg(a, b, c)
        if turn >= keep_turn_deg or accum >= sample_spacing:
            must_keep.add(i)
            accum = 0.0

    out_idx = [0]
    anchor = 0
    while anchor < len(path) - 1:
        nearest_must_keep = len(path) - 1
        for m in range(anchor + 1, len(path)):
            if m in must_keep:
                nearest_must_keep = m
                break

        best = anchor + 1
        probe = anchor + 1
        while probe <= nearest_must_keep:
            if _can_shortcut(
                path,
                nodes,
                anchor,
                probe,
                max_direct_dist2d=max_direct_dist2d,
                max_direct_up_dz=max_direct_up_dz,
                max_direct_abs_dz=max_direct_abs_dz,
                corridor_width2d=corridor_width2d,
            ):
                best = probe
                probe += 1
                continue
            break

        out_idx.append(best)
        anchor = best

    # De-dup in case repeated nodes appear due to degenerate raw paths.
    compact: list[int] = []
    for idx in out_idx:
        node = path[idx]
        if not compact or compact[-1] != node:
            compact.append(node)
    return compact


def _yaw(a: list[float], b: list[float]) -> float:
    return math.degrees(math.atan2(b[1] - a[1], b[0] - a[0]))


def _prune_redundant_long_edges(
    out_adj: dict[int, set[int]],
    nodes: list[list[float]],
    max_long_edge_dist2d: float,
    alt_max_hops: int,
    alt_max_cost_factor: float,
) -> None:
    edges = [(u, v) for u, vs in out_adj.items() for v in vs if u != v]
    edges.sort(key=lambda uv: _dist2d(nodes[uv[0]], nodes[uv[1]]), reverse=True)

    for u, v in edges:
        if v not in out_adj.get(u, set()):
            continue
        direct = _dist2d(nodes[u], nodes[v])
        if direct <= max_long_edge_dist2d:
            continue

        # Temporarily drop direct edge; keep it only if no reasonable alternate walk.
        out_adj[u].discard(v)
        best_cost = 1e30
        q: list[tuple[int, int, float]] = [(u, 0, 0.0)]
        seen: dict[tuple[int, int], float] = {(u, 0): 0.0}
        found = False
        while q:
            cur, hops, cost = q.pop(0)
            if hops >= alt_max_hops:
                continue
            for nxt in out_adj.get(cur, set()):
                nd = cost + _dist2d(nodes[cur], nodes[nxt])
                if nd > direct * alt_max_cost_factor:
                    continue
                if nxt == v:
                    best_cost = min(best_cost, nd)
                    found = True
                    continue
                state = (nxt, hops + 1)
                if nd >= seen.get(state, 1e30):
                    continue
                seen[state] = nd
                q.append((nxt, hops + 1, nd))

        if not found:
            out_adj[u].add(v)


def _split_long_edges(
    nodes: list[list[float]],
    out_adj: dict[int, set[int]],
    max_seg_len2d: float,
) -> tuple[list[list[float]], dict[int, set[int]]]:
    new_nodes = [list(p) for p in nodes]
    new_adj: dict[int, set[int]] = defaultdict(set)
    for u, vs in out_adj.items():
        new_adj[u].update(vs)

    pairs: set[tuple[int, int]] = set()
    for u, vs in list(new_adj.items()):
        for v in list(vs):
            if u == v:
                continue
            pairs.add((min(u, v), max(u, v)))

    for a, b in sorted(pairs):
        pa = new_nodes[a]
        pb = new_nodes[b]
        dist = _dist2d(pa, pb)
        if dist <= max_seg_len2d:
            continue

        seg_count = int(math.ceil(dist / max_seg_len2d))
        if seg_count <= 1:
            continue

        chain = [a]
        for s in range(1, seg_count):
            t = float(s) / float(seg_count)
            np = [
                pa[0] + (pb[0] - pa[0]) * t,
                pa[1] + (pb[1] - pa[1]) * t,
                pa[2] + (pb[2] - pa[2]) * t,
            ]
            ni = len(new_nodes)
            new_nodes.append(np)
            chain.append(ni)
        chain.append(b)

        has_ab = b in new_adj.get(a, set())
        has_ba = a in new_adj.get(b, set())
        if not has_ab and not has_ba:
            continue

        if has_ab:
            new_adj[a].discard(b)
            for u, v in zip(chain, chain[1:]):
                new_adj[u].add(v)
        if has_ba:
            new_adj[b].discard(a)
            rev = list(reversed(chain))
            for u, v in zip(rev, rev[1:]):
                new_adj[u].add(v)

    return new_nodes, new_adj


def generate_manual_paths(input_path: Path, output_path: Path) -> tuple[int, int]:
    old_n = 0
    if input_path.exists():
        try:
            src = json.loads(input_path.read_text(encoding="utf-8"))
            src_wps = src.get("waypoints", [])
            if isinstance(src_wps, list):
                old_n = len(src_wps)
        except Exception:
            old_n = 0

    # Fully new handcrafted Mirage backbone. No dependency on existing graph topology.
    node_specs: list[tuple[str, tuple[float, float, float]]] = [
        ("ct_spawn", (-1023.34, -2511.76, -162.238)),
        ("ct_lane", (-724.614, -2218.64, -178.59)),
        ("a_ticket", (-576.64, -1676.36, -173.204)),
        ("a_ramp_ct", (-391.68, -1682.03, -167.683)),
        ("a_default_back", (-672.001, -1398.29, -166.969)),
        ("a_default", (-676.649, -1195.46, -167.969)),
        ("a_stairs", (-583.667, -1150.95, -167.229)),
        ("palace_entry", (-333.972, -917.759, -166.539)),
        ("connector", (-125.085, -917.222, -165.564)),
        ("mid_lower", (59.315, -849.798, -165.732)),
        ("top_mid", (476.053, -759.288, -157.535)),
        ("mid_window", (507.809, -516.25, -158.969)),
        ("catwalk", (475.187, -370.439, -160.112)),
        ("short", (421.578, -183.583, -164.571)),
        ("b_short", (246.123, -43.181, -174.268)),
        ("underpass", (-97.067, -720.895, -217.766)),
        ("underpass_exit", (54.557, -377.932, -179.292)),
        ("b_apps", (-324.736, -381.889, -166.875)),
        ("upper_tunnel", (-662.802, -422.54, -164.527)),
        ("upper_tunnel_exit", (-820.784, -328.388, -166.968)),
        ("market_door", (-821.611, 91.848, -168.045)),
        ("market_corner", (-873.103, 195.225, -170.401)),
        ("b_entrance", (-791.14, 232.989, -171.463)),
        ("b_site", (-505.939, 421.662, -165.307)),
        ("market_inside", (-1111.92, 242.622, -168.523)),
        ("mid_t_link", (-1011.57, 273.66, -167.199)),
        ("t_spawn_near", (-1044.3, -1483.36, -164.433)),
        ("t_spawn_main", (-1139.4, -1424.43, -164.589)),
        ("t_spawn_far", (-1231.69, -1337.95, -168.073)),
        ("t_ramp", (-1248.6, -1435.58, -158.893)),
    ]
    name_to_idx = {name: i for i, (name, _) in enumerate(node_specs)}
    nodes = [[float(x), float(y), float(z)] for _, (x, y, z) in node_specs]

    logical_links = [
        ("ct_spawn", "ct_lane"),
        ("ct_lane", "a_ticket"),
        ("a_ticket", "a_ramp_ct"),
        ("a_ramp_ct", "a_default_back"),
        ("a_default_back", "a_default"),
        ("a_default", "a_stairs"),
        ("a_stairs", "palace_entry"),
        ("palace_entry", "connector"),
        ("connector", "mid_lower"),
        ("mid_lower", "top_mid"),
        ("top_mid", "mid_window"),
        ("mid_window", "catwalk"),
        ("catwalk", "short"),
        ("short", "b_short"),
        ("connector", "underpass"),
        ("underpass", "underpass_exit"),
        ("underpass_exit", "b_apps"),
        ("b_apps", "upper_tunnel"),
        ("upper_tunnel", "upper_tunnel_exit"),
        ("upper_tunnel_exit", "market_door"),
        ("market_door", "market_corner"),
        ("market_corner", "b_entrance"),
        ("b_entrance", "b_site"),
        ("market_corner", "market_inside"),
        ("market_inside", "mid_t_link"),
        ("mid_t_link", "market_door"),
        ("a_default_back", "t_spawn_near"),
        ("t_spawn_near", "t_spawn_main"),
        ("t_spawn_main", "t_spawn_far"),
        ("t_spawn_far", "t_ramp"),
        ("t_ramp", "a_default_back"),
        ("mid_lower", "underpass"),
    ]

    max_up_dz = 22.0
    out_adj: dict[int, set[int]] = defaultdict(set)
    for a_name, b_name in logical_links:
        a = name_to_idx[a_name]
        b = name_to_idx[b_name]
        pa = nodes[a]
        pb = nodes[b]
        if (pb[2] - pa[2]) <= max_up_dz:
            out_adj[a].add(b)
        if (pa[2] - pb[2]) <= max_up_dz:
            out_adj[b].add(a)

    out_wps: list[dict] = []
    for i, pos in enumerate(nodes):
        nxt = sorted(out_adj.get(i, set()))
        yaw = _yaw(pos, nodes[nxt[0]]) if nxt else 0.0
        out_wps.append(
            {
                "pos": [round(float(pos[0]), 3), round(float(pos[1]), 3), round(float(pos[2]), 3)],
                "angle": [0.0, round(float(yaw), 3), 0.0],
                "type": 1 if nxt else 0,
                "branch_mode": 2 if len(nxt) > 1 else 0,
                "next_indices": nxt,
            }
        )

    output_path.write_text(json.dumps({"waypoints": out_wps}, ensure_ascii=False, indent=2), encoding="utf-8")
    return old_n, len(out_wps)


def main() -> int:
    parser = argparse.ArgumentParser(description="Generate a handcrafted Mirage route graph from existing walkable paths.")
    parser.add_argument("--input", default="CS2_WalkBot_Ext/paths.json")
    parser.add_argument("--output", default="CS2_WalkBot_Ext/paths.json")
    args = parser.parse_args()

    old_n, new_n = generate_manual_paths(Path(args.input), Path(args.output))
    print(f"[+] Rewrote paths manually-inspired: {old_n} -> {new_n}")
    print(f"[+] Wrote: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
