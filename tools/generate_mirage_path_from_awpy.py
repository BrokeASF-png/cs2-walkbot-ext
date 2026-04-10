#!/usr/bin/env python3
import argparse
import json
import math
import sys
from collections import defaultdict
from pathlib import Path


def _norm(s: str) -> str:
    return "".join(ch for ch in s.lower() if ch.isalnum())


def _center(area: dict) -> tuple[float, float, float]:
    x = (float(area["northWestX"]) + float(area["southEastX"])) * 0.5
    y = (float(area["northWestY"]) + float(area["southEastY"])) * 0.5
    z = (float(area["northWestZ"]) + float(area["southEastZ"])) * 0.5
    return x, y, z


def _yaw(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    return math.degrees(math.atan2(dy, dx))

def _dist2d(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    return math.hypot(dx, dy)


def _turn_deg(
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    c: tuple[float, float, float],
) -> float:
    v1x = b[0] - a[0]
    v1y = b[1] - a[1]
    v2x = c[0] - b[0]
    v2y = c[1] - b[1]
    l1 = math.hypot(v1x, v1y)
    l2 = math.hypot(v2x, v2y)
    if l1 < 1e-5 or l2 < 1e-5:
        return 0.0
    dot = (v1x * v2x + v1y * v2y) / (l1 * l2)
    dot = max(-1.0, min(1.0, dot))
    return math.degrees(math.acos(dot))


TOKEN_GROUPS = {
    "t_spawn": [["tspawn"], ["terroristspawn"], ["tstart"]],
    "t_ramp": [["tramp"], ["ramp"]],
    "top_mid": [["topmid"], ["topofmid"]],
    "mid": [["mid"], ["middle"]],
    "mid_boxes": [["midbox"], ["boxes"]],
    "underpass": [["underpass"]],
    "connector": [["connector"], ["con"]],
    "jungle": [["jungle"]],
    "window": [["window"], ["sniper"]],
    "catwalk": [["catwalk"], ["short"]],
    "short_to_b": [["shortb"], ["bshort"]],
    "a_ramp": [["aramp"], ["a ramp"]],
    "a_site": [["asite"], ["bombsitea"], ["sitea"]],
    "palace_hall": [["palace"], ["palacehall"]],
    "palace_balcony": [["palacebalcony"], ["balcony"]],
    "upper_tunnel": [["uppertunnel"], ["tunnel"], ["tunnels"]],
    "b_apps": [["apartments"], ["bapps"], ["apps"]],
    "b_balcony": [["bbalcony"], ["balconyb"]],
    "b_site": [["bsite"], ["bombsiteb"], ["siteb"]],
    "market": [["market"]],
    "ct_spawn": [["ctspawn"], ["counterterroristspawn"], ["ctstart"]],
    "ticket": [["ticket"]],
}


ROUTE_CHAINS = [
    ["t_spawn", "t_ramp", "top_mid", "mid", "connector", "a_site"],
    ["t_spawn", "t_ramp", "top_mid", "underpass", "connector", "a_site"],
    ["t_spawn", "palace_hall", "palace_balcony", "a_site"],
    ["t_spawn", "t_ramp", "a_ramp", "a_site"],
    ["t_spawn", "upper_tunnel", "b_apps", "b_balcony", "b_site"],
    ["t_spawn", "upper_tunnel", "b_apps", "catwalk", "short_to_b", "b_site"],
    ["t_spawn", "t_ramp", "top_mid", "catwalk", "short_to_b", "b_site"],
    ["ct_spawn", "ticket", "a_site"],
    ["ct_spawn", "jungle", "connector", "mid"],
    ["ct_spawn", "window", "mid"],
    ["ct_spawn", "market", "b_site"],
    ["ct_spawn", "short_to_b", "b_site"],
    # Extra centerline chains to keep full-map connectivity with simple trunks.
    ["ct_spawn", "mid", "top_mid", "t_spawn"],
    ["ct_spawn", "mid", "underpass", "upper_tunnel", "b_site"],
    ["ct_spawn", "connector", "jungle", "a_site"],
]


def _find_callout_nodes(map_nav: dict) -> dict[str, list[int]]:
    by_name = defaultdict(list)
    for k, area in map_nav.items():
        name = _norm(str(area.get("areaName", "")))
        by_name[name].append(int(k))

    callout_nodes: dict[str, list[int]] = {}
    all_names = list(by_name.keys())

    for callout, token_sets in TOKEN_GROUPS.items():
        matched: list[int] = []
        for name in all_names:
            for token_set in token_sets:
                tokens = [_norm(t) for t in token_set]
                if all(tok in name for tok in tokens):
                    matched.extend(by_name[name])
                    break
        # De-dup, stable order by id.
        if matched:
            callout_nodes[callout] = sorted(set(matched))

    return callout_nodes


def _pick_anchor(graph, node_ids: list[int]) -> int:
    # Prefer highly connected area for stable shortest path behavior.
    best = node_ids[0]
    best_deg = -1
    for nid in node_ids:
        deg = graph.out_degree(nid) + graph.in_degree(nid)
        if deg > best_deg:
            best = nid
            best_deg = deg
    return best


def _shortest_path(graph, a: int, b: int) -> list[int]:
    try:
        return graph.shortest_path(a, b, weight="weight")
    except Exception:
        # networkx graph object from awpy is usually nx.DiGraph;
        # keep broad fallback for compatibility.
        import networkx as nx
        return nx.shortest_path(graph, a, b, weight="weight")

def _compress_path_polyline(
    path: list[int],
    centers: dict[int, tuple[float, float, float]],
    keep_turn_deg: float,
    sample_spacing: float,
    max_vertical_step: float,
) -> list[int]:
    if len(path) <= 2:
        return path

    kept: list[int] = [path[0]]
    accum = 0.0

    for i in range(1, len(path) - 1):
        prev_kept = kept[-1]
        cur = path[i]
        nxt = path[i + 1]
        a = centers[prev_kept]
        b = centers[cur]
        c = centers[nxt]

        accum += _dist2d(a, b)
        turn = _turn_deg(a, b, c)
        dz = abs(c[2] - a[2])

        should_keep = False
        if turn >= keep_turn_deg:
            should_keep = True
        elif accum >= sample_spacing:
            should_keep = True
        elif dz >= max_vertical_step:
            should_keep = True

        if should_keep:
            kept.append(cur)
            accum = 0.0

    if kept[-1] != path[-1]:
        kept.append(path[-1])
    return kept


def _merge_close_nodes(
    used_nodes: set[int],
    used_edges: set[tuple[int, int]],
    centers: dict[int, tuple[float, float, float]],
    merge_radius_xy: float,
) -> tuple[set[int], set[tuple[int, int]], dict[int, int]]:
    if not used_nodes:
        return used_nodes, used_edges, {}

    sorted_nodes = sorted(used_nodes)
    rep_of: dict[int, int] = {}
    reps: list[int] = []

    for nid in sorted_nodes:
        p = centers[nid]
        chosen = -1
        for rep in reps:
            rp = centers[rep]
            if _dist2d(p, rp) <= merge_radius_xy and abs(p[2] - rp[2]) <= 48.0:
                chosen = rep
                break
        if chosen < 0:
            reps.append(nid)
            chosen = nid
        rep_of[nid] = chosen

    merged_nodes: set[int] = set(rep_of[n] for n in used_nodes)
    merged_edges: set[tuple[int, int]] = set()
    for u, v in used_edges:
        ru = rep_of.get(u, u)
        rv = rep_of.get(v, v)
        if ru == rv:
            continue
        merged_edges.add((ru, rv))

    return merged_nodes, merged_edges, rep_of


def generate(
    output_file: Path,
    jump_up_limit: float,
    keep_turn_deg: float,
    sample_spacing: float,
    vertical_keep_step: float,
    merge_radius_xy: float,
) -> int:
    try:
        from awpy.data import NAV, NAV_GRAPHS
    except Exception as e:
        print(f"[!] Failed to import awpy: {e}")
        print("[*] Install awpy first: pip install awpy")
        return 2

    if "de_mirage" not in NAV or "de_mirage" not in NAV_GRAPHS:
        print("[!] de_mirage nav data not found in awpy cache.")
        print("[*] Run: python -m awpy.cli artifacts --resource navs --patch current")
        return 3

    map_nav = NAV["de_mirage"]
    graph = NAV_GRAPHS["de_mirage"]
    callout_nodes = _find_callout_nodes(map_nav)

    missing = [c for c in set(sum(ROUTE_CHAINS, [])) if c not in callout_nodes]
    if missing:
        print("[!] Missing callouts from nav data:", ", ".join(sorted(missing)))
        print("[*] This usually means callout naming changed in this patch.")
        return 4

    anchors = {k: _pick_anchor(graph, v) for k, v in callout_nodes.items()}

    used_edges: set[tuple[int, int]] = set()
    used_nodes: set[int] = set()
    centers = {int(k): _center(v) for k, v in map_nav.items()}

    for chain in ROUTE_CHAINS:
        for i in range(len(chain) - 1):
            a = anchors[chain[i]]
            b = anchors[chain[i + 1]]
            raw_path = _shortest_path(graph, a, b)
            if len(raw_path) < 2:
                continue
            path = _compress_path_polyline(
                raw_path,
                centers,
                keep_turn_deg=keep_turn_deg,
                sample_spacing=sample_spacing,
                max_vertical_step=vertical_keep_step,
            )
            for u, v in zip(path, path[1:]):
                zu = centers[u][2]
                zv = centers[v][2]
                # Keep drop-down one-way edges, remove jump-up edges.
                if (zv - zu) > jump_up_limit:
                    continue
                used_edges.add((u, v))
                used_nodes.add(u)
                used_nodes.add(v)

    if not used_nodes:
        print("[!] No nodes selected. Check route configuration.")
        return 5

    used_nodes, used_edges, _ = _merge_close_nodes(
        used_nodes,
        used_edges,
        centers,
        merge_radius_xy=merge_radius_xy,
    )

    node_list = sorted(used_nodes)
    index_of = {nid: i for i, nid in enumerate(node_list)}
    outgoing = defaultdict(list)
    for u, v in used_edges:
        if u in index_of and v in index_of:
            outgoing[index_of[u]].append(index_of[v])

    waypoints = []
    for i, nid in enumerate(node_list):
        pos = centers[nid]
        nxt = sorted(set(outgoing.get(i, [])))
        if nxt:
            yaw = _yaw(pos, centers[node_list[nxt[0]]])
        else:
            yaw = 0.0
        wp_type = 1 if nxt else 0
        wp = {
            "pos": [round(pos[0], 3), round(pos[1], 3), round(pos[2], 3)],
            "angle": [0.0, round(yaw, 3), 0.0],
            "type": wp_type,
            "branch_mode": 2 if len(nxt) > 1 else 0,
            "next_indices": nxt,
        }
        waypoints.append(wp)

    output = {"waypoints": waypoints}
    output_file.parent.mkdir(parents=True, exist_ok=True)
    output_file.write_text(json.dumps(output, indent=2), encoding="utf-8")
    print(f"[+] Wrote {len(waypoints)} waypoints to {output_file}")
    print(f"[+] Jump-up filter: removed edges with dz > {jump_up_limit}")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Generate Mirage paths.json centerline-style: straighter, fewer points, full-map coverage."
    )
    parser.add_argument(
        "--output",
        default="CS2_WalkBot_Ext/paths.json",
        help="Output paths.json path",
    )
    parser.add_argument(
        "--jump-up-limit",
        type=float,
        default=18.0,
        help="Max allowed upward dz per edge (units)",
    )
    parser.add_argument(
        "--keep-turn-deg",
        type=float,
        default=22.0,
        help="Keep nodes on turns sharper than this angle",
    )
    parser.add_argument(
        "--sample-spacing",
        type=float,
        default=190.0,
        help="Keep one node every N XY units on straights",
    )
    parser.add_argument(
        "--vertical-keep-step",
        type=float,
        default=24.0,
        help="Keep nodes when vertical step accumulation is notable",
    )
    parser.add_argument(
        "--merge-radius-xy",
        type=float,
        default=72.0,
        help="Merge nearby overlapping nodes across routes",
    )
    args = parser.parse_args()
    return generate(
        Path(args.output),
        args.jump_up_limit,
        args.keep_turn_deg,
        args.sample_spacing,
        args.vertical_keep_step,
        args.merge_radius_xy,
    )


if __name__ == "__main__":
    sys.exit(main())
