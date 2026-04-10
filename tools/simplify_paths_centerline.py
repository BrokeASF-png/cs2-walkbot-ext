#!/usr/bin/env python3
from __future__ import annotations

import argparse
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


def _build_graph(waypoints: list[dict]) -> tuple[dict[int, set[int]], dict[int, set[int]]]:
    out_adj: dict[int, set[int]] = defaultdict(set)
    in_adj: dict[int, set[int]] = defaultdict(set)
    for i, wp in enumerate(waypoints):
        for n in wp.get("next_indices", []):
            if isinstance(n, int) and 0 <= n < len(waypoints) and n != i:
                out_adj[i].add(n)
                in_adj[n].add(i)
    return out_adj, in_adj


def _reindex_waypoints_with_edges(waypoints: list[dict], out_adj: dict[int, set[int]]) -> list[dict]:
    keep = sorted(
        i for i in range(len(waypoints))
        if out_adj.get(i) or any(i in out_adj.get(k, set()) for k in out_adj.keys())
    )
    if not keep:
        keep = list(range(len(waypoints)))
    new_of = {old: ni for ni, old in enumerate(keep)}
    out_new: dict[int, set[int]] = defaultdict(set)
    for old_u in keep:
        nu = new_of[old_u]
        for old_v in out_adj.get(old_u, set()):
            if old_v in new_of and old_v != old_u:
                out_new[nu].add(new_of[old_v])

    new_wps: list[dict] = []
    for old_i in keep:
        ni = new_of[old_i]
        wp = waypoints[old_i]
        nxt = sorted(out_new.get(ni, set()))
        new_wps.append(
            {
                "pos": [round(float(wp["pos"][0]), 3), round(float(wp["pos"][1]), 3), round(float(wp["pos"][2]), 3)],
                "angle": [round(float(wp["angle"][0]), 3), round(float(wp["angle"][1]), 3), round(float(wp["angle"][2]), 3)],
                "type": 1 if nxt else 0,
                "branch_mode": 2 if len(nxt) > 1 else 0,
                "next_indices": nxt,
            }
        )
    _recompute_yaw(new_wps)
    return new_wps


def _filter_unwalkable_jump_edges(
    waypoints: list[dict],
    max_walk_up_dz: float,
    max_walk_abs_dz: float,
    max_walk_grade: float,
    min_ramp_dist2d: float,
    hard_max_up_dz: float,
    hard_max_abs_dz: float,
    allow_slope_chain_exception: bool,
    slope_chain_min_dist2d: float,
    slope_chain_max_grade: float,
    slope_chain_max_sign_changes: int,
    slope_chain_edge_up_dz: float,
    slope_chain_edge_abs_dz: float,
) -> list[dict]:
    out_adj, _ = _build_graph(waypoints)
    in_adj: dict[int, set[int]] = defaultdict(set)
    for u, vs in out_adj.items():
        for v in vs:
            in_adj[v].add(u)

    slope_chain_edges: set[tuple[int, int]] = set()
    if allow_slope_chain_exception:
        visited_chain_edge: set[tuple[int, int]] = set()

        def extract_chain_from(start_u: int, start_v: int) -> list[int]:
            chain = [start_u, start_v]
            cur = start_v
            while len(out_adj.get(cur, set())) == 1 and len(in_adj.get(cur, set())) == 1:
                nxt = next(iter(out_adj[cur]))
                if nxt in chain:
                    break
                edge = (cur, nxt)
                if edge in visited_chain_edge:
                    break
                chain.append(nxt)
                cur = nxt
            return chain

        for u in range(len(waypoints)):
            if len(out_adj.get(u, set())) == 0:
                continue
            is_start = len(in_adj.get(u, set())) != 1 or len(out_adj.get(u, set())) != 1
            if not is_start:
                continue
            for v in sorted(out_adj.get(u, set())):
                if (u, v) in visited_chain_edge:
                    continue
                chain = extract_chain_from(u, v)
                for a, b in zip(chain, chain[1:]):
                    visited_chain_edge.add((a, b))

                if len(chain) < 3:
                    continue
                total_dist2d = 0.0
                sign_changes = 0
                prev_sign = 0
                max_up = 0.0
                max_abs = 0.0
                for a, b in zip(chain, chain[1:]):
                    pa = waypoints[a]["pos"]
                    pb = waypoints[b]["pos"]
                    total_dist2d += _dist2d(pa, pb)
                    dz = pb[2] - pa[2]
                    max_up = max(max_up, dz)
                    max_abs = max(max_abs, abs(dz))
                    sign = 1 if dz > 1.5 else (-1 if dz < -1.5 else 0)
                    if sign != 0:
                        if prev_sign != 0 and sign != prev_sign:
                            sign_changes += 1
                        prev_sign = sign

                if total_dist2d < slope_chain_min_dist2d:
                    continue
                p0 = waypoints[chain[0]]["pos"]
                p1 = waypoints[chain[-1]]["pos"]
                grade = abs(p1[2] - p0[2]) / max(1.0, _dist2d(p0, p1))
                if grade > slope_chain_max_grade:
                    continue
                if sign_changes > slope_chain_max_sign_changes:
                    continue
                if max_up > slope_chain_edge_up_dz or max_abs > slope_chain_edge_abs_dz:
                    continue

                for a, b in zip(chain, chain[1:]):
                    slope_chain_edges.add((a, b))

    filtered: dict[int, set[int]] = defaultdict(set)
    for u in range(len(waypoints)):
        pu = waypoints[u]["pos"]
        for v in out_adj.get(u, set()):
            pv = waypoints[v]["pos"]
            up_dz = pv[2] - pu[2]
            abs_dz = abs(up_dz)
            d2 = _dist2d(pu, pv)
            grade = abs_dz / max(1.0, d2)
            # Normal flat/step rule.
            normal_ok = up_dz <= max_walk_up_dz and abs_dz <= max_walk_abs_dz
            # Ramp rule: allow larger dz when slope is gentle and segment is long enough.
            ramp_ok = (
                d2 >= min_ramp_dist2d and
                grade <= max_walk_grade and
                up_dz <= hard_max_up_dz and
                abs_dz <= hard_max_abs_dz
            )
            slope_ok = (u, v) in slope_chain_edges
            if not normal_ok and not ramp_ok and not slope_ok:
                continue
            filtered[u].add(v)
    return _reindex_waypoints_with_edges(waypoints, filtered)


def _reconnect_walkable_slopes(
    waypoints: list[dict],
    min_link_dist2d: float,
    max_link_dist2d: float,
    max_link_up_dz: float,
    max_link_abs_dz: float,
    max_grade: float,
    hard_max_up_dz: float,
    hard_max_abs_dz: float,
    max_new_links_per_node: int,
) -> list[dict]:
    out_adj, _ = _build_graph(waypoints)
    n = len(waypoints)
    und: list[set[int]] = [set() for _ in range(n)]
    for u in range(n):
        for v in out_adj.get(u, set()):
            und[u].add(v)
            und[v].add(u)

    comp_id = [-1] * n
    cid = 0
    for i in range(n):
        if comp_id[i] != -1:
            continue
        stack = [i]
        comp_id[i] = cid
        while stack:
            u = stack.pop()
            for v in und[u]:
                if comp_id[v] == -1:
                    comp_id[v] = cid
                    stack.append(v)
        cid += 1

    if cid <= 1:
        return waypoints

    cell = max(16.0, max_link_dist2d)
    buckets: dict[tuple[int, int], list[int]] = defaultdict(list)
    for i, wp in enumerate(waypoints):
        x, y, _ = wp["pos"]
        buckets[(int(math.floor(x / cell)), int(math.floor(y / cell)))].append(i)

    new_out: dict[int, set[int]] = {i: set(out_adj.get(i, set())) for i in range(n)}
    new_links_per_node = [0] * n

    for u in range(n):
        pu = waypoints[u]["pos"]
        cx = int(math.floor(pu[0] / cell))
        cy = int(math.floor(pu[1] / cell))
        candidates: list[tuple[float, int]] = []
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for v in buckets.get((cx + dx, cy + dy), []):
                    if v == u or comp_id[v] == comp_id[u]:
                        continue
                    pv = waypoints[v]["pos"]
                    d2 = _dist2d(pu, pv)
                    if d2 < min_link_dist2d or d2 > max_link_dist2d:
                        continue
                    dz = pv[2] - pu[2]
                    if abs(dz) > max_link_abs_dz:
                        continue
                    grade = abs(dz) / max(1.0, d2)
                    if grade > max_grade:
                        continue
                    if dz > hard_max_up_dz or abs(dz) > hard_max_abs_dz:
                        continue
                    candidates.append((d2 + abs(dz) * 0.5, v))
        if not candidates:
            continue

        candidates.sort(key=lambda x: x[0])
        for _, v in candidates:
            if new_links_per_node[u] >= max_new_links_per_node:
                break
            if v in new_out[u]:
                continue
            pu2 = waypoints[u]["pos"]
            pv2 = waypoints[v]["pos"]
            dz_uv = pv2[2] - pu2[2]
            dz_vu = -dz_uv

            if dz_uv <= max_link_up_dz:
                new_out[u].add(v)
                new_links_per_node[u] += 1
            if dz_vu <= max_link_up_dz and new_links_per_node[v] < max_new_links_per_node:
                new_out[v].add(u)
                new_links_per_node[v] += 1

    return _reindex_waypoints_with_edges(waypoints, new_out)


class _UnionFind:
    def __init__(self, n: int) -> None:
        self.p = list(range(n))
        self.r = [0] * n

    def find(self, x: int) -> int:
        while self.p[x] != x:
            self.p[x] = self.p[self.p[x]]
            x = self.p[x]
        return x

    def union(self, a: int, b: int) -> None:
        ra = self.find(a)
        rb = self.find(b)
        if ra == rb:
            return
        if self.r[ra] < self.r[rb]:
            ra, rb = rb, ra
        self.p[rb] = ra
        if self.r[ra] == self.r[rb]:
            self.r[ra] += 1


def _merge_close_nodes(
    waypoints: list[dict],
    merge_radius_xy: float,
    merge_radius_z: float,
) -> list[dict]:
    order = sorted(range(len(waypoints)), key=lambda i: len(waypoints[i].get("next_indices", [])), reverse=True)
    reps: list[int] = []
    rep_of: dict[int, int] = {}

    for idx in order:
        p = waypoints[idx]["pos"]
        chosen = -1
        for r in reps:
            rp = waypoints[r]["pos"]
            if _dist2d(p, rp) <= merge_radius_xy and abs(p[2] - rp[2]) <= merge_radius_z:
                chosen = r
                break
        if chosen < 0:
            chosen = idx
            reps.append(idx)
        rep_of[idx] = chosen

    rep_list = sorted(set(rep_of.values()))
    new_index_of = {old: i for i, old in enumerate(rep_list)}

    merged_out: dict[int, set[int]] = defaultdict(set)
    for old_i, wp in enumerate(waypoints):
        ni = new_index_of[rep_of[old_i]]
        for old_j in wp.get("next_indices", []):
            if old_j not in rep_of:
                continue
            nj = new_index_of[rep_of[old_j]]
            if ni != nj:
                merged_out[ni].add(nj)

    new_waypoints: list[dict] = []
    for old_rep in rep_list:
        ni = new_index_of[old_rep]
        pos = waypoints[old_rep]["pos"]
        ang = waypoints[old_rep].get("angle", [0.0, 0.0, 0.0])
        nxt = sorted(merged_out.get(ni, set()))
        new_waypoints.append(
            {
                "pos": [round(float(pos[0]), 3), round(float(pos[1]), 3), round(float(pos[2]), 3)],
                "angle": [round(float(ang[0]), 3), round(float(ang[1]), 3), round(float(ang[2]), 3)],
                "type": 1 if nxt else 0,
                "branch_mode": 2 if len(nxt) > 1 else 0,
                "next_indices": nxt,
            }
        )

    return new_waypoints


def _merge_nearby_centroids(
    waypoints: list[dict],
    merge_radius_xy: float,
    merge_radius_z: float,
    passes: int,
) -> list[dict]:
    for _ in range(max(1, passes)):
        n = len(waypoints)
        if n <= 1:
            break
        uf = _UnionFind(n)

        # Spatial hash buckets to reduce O(n^2).
        cell = max(8.0, merge_radius_xy)
        buckets: dict[tuple[int, int], list[int]] = defaultdict(list)
        for i, wp in enumerate(waypoints):
            x, y, _ = wp["pos"]
            buckets[(int(math.floor(x / cell)), int(math.floor(y / cell)))].append(i)

        for i, wp in enumerate(waypoints):
            x, y, z = wp["pos"]
            cx = int(math.floor(x / cell))
            cy = int(math.floor(y / cell))
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    for j in buckets.get((cx + dx, cy + dy), []):
                        if j <= i:
                            continue
                        x2, y2, z2 = waypoints[j]["pos"]
                        if abs(z2 - z) > merge_radius_z:
                            continue
                        if math.hypot(x2 - x, y2 - y) <= merge_radius_xy:
                            uf.union(i, j)

        groups: dict[int, list[int]] = defaultdict(list)
        for i in range(n):
            groups[uf.find(i)].append(i)
        if len(groups) == n:
            break

        old_to_new: dict[int, int] = {}
        new_waypoints: list[dict] = []
        for members in groups.values():
            sx = sy = sz = 0.0
            for m in members:
                p = waypoints[m]["pos"]
                sx += float(p[0])
                sy += float(p[1])
                sz += float(p[2])
            c = float(len(members))
            ni = len(new_waypoints)
            for m in members:
                old_to_new[m] = ni
            new_waypoints.append(
                {
                    "pos": [round(sx / c, 3), round(sy / c, 3), round(sz / c, 3)],
                    "angle": [0.0, 0.0, 0.0],
                    "type": 0,
                    "branch_mode": 0,
                    "next_indices": [],
                }
            )

        out_new: dict[int, set[int]] = defaultdict(set)
        for i, wp in enumerate(waypoints):
            ni = old_to_new[i]
            for j in wp.get("next_indices", []):
                if not isinstance(j, int) or j < 0 or j >= n:
                    continue
                nj = old_to_new[j]
                if ni != nj:
                    out_new[ni].add(nj)

        for i, wp in enumerate(new_waypoints):
            nxt = sorted(out_new.get(i, set()))
            wp["next_indices"] = nxt
            wp["type"] = 1 if nxt else 0
            wp["branch_mode"] = 2 if len(nxt) > 1 else 0
        _recompute_yaw(new_waypoints)
        waypoints = new_waypoints

    return waypoints


def _collapse_linear_nodes(
    waypoints: list[dict],
    max_passes: int,
    turn_keep_deg: float,
    max_merge_seg_2d: float,
    max_merge_dz: float,
) -> list[dict]:
    for _ in range(max(0, max_passes)):
        out_adj, in_adj = _build_graph(waypoints)
        removed = set()
        rewired: dict[int, set[int]] = defaultdict(set)
        changed = False

        for n in range(len(waypoints)):
            if n in removed:
                continue
            ins = sorted(in_adj.get(n, set()))
            outs = sorted(out_adj.get(n, set()))
            if len(ins) != 1 or len(outs) != 1:
                continue
            p = ins[0]
            s = outs[0]
            if p == n or s == n or p == s:
                continue
            if p in removed or s in removed:
                continue

            pp = waypoints[p]["pos"]
            nn = waypoints[n]["pos"]
            ss = waypoints[s]["pos"]
            turn = _turn_deg(pp, nn, ss)
            dps = _dist2d(pp, ss)
            dz = abs(ss[2] - pp[2])

            # Keep corners and long cross-links to avoid wall-cut shortcuts.
            if turn >= turn_keep_deg:
                continue
            if dps > max_merge_seg_2d:
                continue
            if dz > max_merge_dz:
                continue

            removed.add(n)
            rewired[p].add(s)
            changed = True

        if not changed:
            break

        kept_old = [i for i in range(len(waypoints)) if i not in removed]
        new_idx = {old: i for i, old in enumerate(kept_old)}

        # Carry old edges except edges touching removed nodes.
        out_new: dict[int, set[int]] = defaultdict(set)
        for old_i in kept_old:
            ni = new_idx[old_i]
            for old_j in waypoints[old_i].get("next_indices", []):
                if old_j in removed:
                    continue
                if old_j in new_idx and old_j != old_i:
                    out_new[ni].add(new_idx[old_j])

        # Apply rewires.
        for p, succs in rewired.items():
            if p not in new_idx:
                continue
            np = new_idx[p]
            for s in succs:
                if s not in new_idx:
                    continue
                ns = new_idx[s]
                if np != ns:
                    out_new[np].add(ns)

        new_wps: list[dict] = []
        for old_i in kept_old:
            ni = new_idx[old_i]
            wp = waypoints[old_i]
            nxt = sorted(out_new.get(ni, set()))
            new_wps.append(
                {
                    "pos": [round(float(wp["pos"][0]), 3), round(float(wp["pos"][1]), 3), round(float(wp["pos"][2]), 3)],
                    "angle": [round(float(wp["angle"][0]), 3), round(float(wp["angle"][1]), 3), round(float(wp["angle"][2]), 3)],
                    "type": 1 if nxt else 0,
                    "branch_mode": 2 if len(nxt) > 1 else 0,
                    "next_indices": nxt,
                }
            )
        waypoints = new_wps

    return waypoints


def _recompute_yaw(waypoints: list[dict]) -> None:
    for i, wp in enumerate(waypoints):
        nxt = wp.get("next_indices", [])
        if not nxt:
            wp["angle"] = [0.0, wp["angle"][1] if "angle" in wp and len(wp["angle"]) > 1 else 0.0, 0.0]
            continue
        a = wp["pos"]
        b = waypoints[nxt[0]]["pos"]
        yaw = math.degrees(math.atan2(b[1] - a[1], b[0] - a[0]))
        wp["angle"] = [0.0, round(yaw, 3), 0.0]
        wp["type"] = 1
        wp["branch_mode"] = 2 if len(nxt) > 1 else 0


def _build_backbone_tree(waypoints: list[dict]) -> list[dict]:
    n = len(waypoints)
    out_adj, _ = _build_graph(waypoints)
    und: list[set[int]] = [set() for _ in range(n)]
    for i in range(n):
        for j in out_adj.get(i, set()):
            if 0 <= j < n and i != j:
                und[i].add(j)
                und[j].add(i)

    visited = [False] * n
    components: list[list[int]] = []
    for i in range(n):
        if visited[i]:
            continue
        stack = [i]
        visited[i] = True
        comp = []
        while stack:
            u = stack.pop()
            comp.append(u)
            for v in und[u]:
                if not visited[v]:
                    visited[v] = True
                    stack.append(v)
        components.append(comp)

    # Build one directed backbone per component:
    # parent -> child for trunk flow, plus leaf -> parent to avoid dead-end lock.
    directed_edges: set[tuple[int, int]] = set()
    for comp in components:
        if len(comp) <= 1:
            continue
        comp_set = set(comp)
        root = max(comp, key=lambda x: len(und[x]))
        in_tree = {root}
        frontier: list[tuple[float, int, int]] = []

        def push_edges(u: int) -> None:
            pu = waypoints[u]["pos"]
            for v in und[u]:
                if v in in_tree or v not in comp_set:
                    continue
                pv = waypoints[v]["pos"]
                w = _dist2d(pu, pv) + abs(pu[2] - pv[2]) * 0.35
                frontier.append((w, u, v))

        push_edges(root)
        parent_of: dict[int, int] = {}

        while len(in_tree) < len(comp):
            if not frontier:
                # disconnected residue (shouldn't happen), connect nearest remaining
                rem = [x for x in comp if x not in in_tree]
                v = rem[0]
                u = min(in_tree, key=lambda k: _dist2d(waypoints[k]["pos"], waypoints[v]["pos"]))
                frontier.append((_dist2d(waypoints[u]["pos"], waypoints[v]["pos"]), u, v))

            frontier.sort(key=lambda x: x[0])
            w, u, v = frontier.pop(0)
            if v in in_tree:
                continue
            in_tree.add(v)
            parent_of[v] = u
            push_edges(v)

        children_of: dict[int, list[int]] = defaultdict(list)
        for child, parent in parent_of.items():
            children_of[parent].append(child)
            directed_edges.add((parent, child))

        for node in comp:
            if node == root:
                continue
            if len(children_of.get(node, [])) == 0:
                parent = parent_of.get(node)
                if parent is not None:
                    directed_edges.add((node, parent))

    new_wps: list[dict] = []
    new_out: dict[int, set[int]] = defaultdict(set)
    for u, v in directed_edges:
        if u != v:
            new_out[u].add(v)

    for i, wp in enumerate(waypoints):
        nxt = sorted(new_out.get(i, set()))
        new_wps.append(
            {
                "pos": [round(float(wp["pos"][0]), 3), round(float(wp["pos"][1]), 3), round(float(wp["pos"][2]), 3)],
                "angle": [round(float(wp["angle"][0]), 3), round(float(wp["angle"][1]), 3), round(float(wp["angle"][2]), 3)],
                "type": 1 if nxt else 0,
                "branch_mode": 2 if len(nxt) > 1 else 0,
                "next_indices": nxt,
            }
        )
    _recompute_yaw(new_wps)
    return new_wps


def _apply_diagonal_shortcuts(
    waypoints: list[dict],
    max_shortcut_dist2d: float,
    max_shortcut_up_dz: float,
    max_shortcut_abs_dz: float,
    corridor_width2d: float,
) -> list[dict]:
    out_adj, in_adj = _build_graph(waypoints)
    new_out: dict[int, set[int]] = {i: set(out_adj.get(i, set())) for i in range(len(waypoints))}

    def can_shortcut(chain_nodes: list[int], ia: int, ib: int) -> bool:
        a = waypoints[chain_nodes[ia]]["pos"]
        b = waypoints[chain_nodes[ib]]["pos"]
        if _dist2d(a, b) > max_shortcut_dist2d:
            return False
        if (b[2] - a[2]) > max_shortcut_up_dz:
            return False
        if abs(b[2] - a[2]) > max_shortcut_abs_dz:
            return False
        for k in range(ia + 1, ib):
            p = waypoints[chain_nodes[k]]["pos"]
            if _point_line_dist2d(a, b, p) > corridor_width2d:
                return False
        return True

    visited = set()
    starts = [i for i in range(len(waypoints)) if len(out_adj.get(i, set())) != 1 or len(in_adj.get(i, set())) != 1]
    for start in starts:
        for nxt in sorted(out_adj.get(start, set())):
            edge_key = (start, nxt)
            if edge_key in visited:
                continue
            chain = [start, nxt]
            visited.add(edge_key)

            cur = nxt
            while len(out_adj.get(cur, set())) == 1 and len(in_adj.get(cur, set())) == 1:
                succ = next(iter(out_adj[cur]))
                if succ in chain:
                    break
                visited.add((cur, succ))
                chain.append(succ)
                cur = succ

            if len(chain) < 3:
                continue

            compressed = [0]
            anchor = 0
            while anchor < len(chain) - 1:
                best = anchor + 1
                probe = best + 1
                while probe < len(chain) and can_shortcut(chain, anchor, probe):
                    best = probe
                    probe += 1
                compressed.append(best)
                anchor = best

            for i in range(len(chain) - 1):
                u = chain[i]
                v = chain[i + 1]
                new_out.setdefault(u, set()).discard(v)

            for i in range(len(compressed) - 1):
                u = chain[compressed[i]]
                v = chain[compressed[i + 1]]
                if u != v:
                    new_out.setdefault(u, set()).add(v)

    new_wps: list[dict] = []
    for i, wp in enumerate(waypoints):
        nxt = sorted(new_out.get(i, set()))
        new_wps.append(
            {
                "pos": [round(float(wp["pos"][0]), 3), round(float(wp["pos"][1]), 3), round(float(wp["pos"][2]), 3)],
                "angle": [round(float(wp["angle"][0]), 3), round(float(wp["angle"][1]), 3), round(float(wp["angle"][2]), 3)],
                "type": 1 if nxt else 0,
                "branch_mode": 2 if len(nxt) > 1 else 0,
                "next_indices": nxt,
            }
        )
    _recompute_yaw(new_wps)
    return new_wps


def _redirect_single_out_straight(
    waypoints: list[dict],
    max_passes: int,
    max_redirect_dist2d: float,
    max_redirect_up_dz: float,
    max_redirect_abs_dz: float,
    max_redirect_turn_deg: float,
) -> list[dict]:
    for _ in range(max(0, max_passes)):
        out_adj, in_adj = _build_graph(waypoints)
        changed = False

        for u in range(len(waypoints)):
            outs_u = sorted(out_adj.get(u, set()))
            if len(outs_u) != 1:
                continue
            v = outs_u[0]
            outs_v = sorted(out_adj.get(v, set()))
            if not outs_v:
                continue

            pu = waypoints[u]["pos"]
            pv = waypoints[v]["pos"]
            best_w = -1
            best_turn = 9999.0

            for w in outs_v:
                if w == u or w == v:
                    continue
                pw = waypoints[w]["pos"]
                d2 = _dist2d(pu, pw)
                if d2 > max_redirect_dist2d:
                    continue
                if (pw[2] - pu[2]) > max_redirect_up_dz:
                    continue
                if abs(pw[2] - pu[2]) > max_redirect_abs_dz:
                    continue
                turn = _turn_deg(pu, pv, pw)
                if turn > max_redirect_turn_deg:
                    continue
                # v should lie close to the direct line u->w, otherwise likely around an obstacle.
                if _point_line_dist2d(pu, pw, pv) > 42.0:
                    continue
                if turn < best_turn:
                    best_turn = turn
                    best_w = w

            if best_w < 0:
                continue

            # Redirect u directly to straighter successor.
            out_adj[u].discard(v)
            out_adj[u].add(best_w)
            changed = True

        if not changed:
            break

        new_wps: list[dict] = []
        for i, wp in enumerate(waypoints):
            nxt = sorted(out_adj.get(i, set()))
            new_wps.append(
                {
                    "pos": [round(float(wp["pos"][0]), 3), round(float(wp["pos"][1]), 3), round(float(wp["pos"][2]), 3)],
                    "angle": [round(float(wp["angle"][0]), 3), round(float(wp["angle"][1]), 3), round(float(wp["angle"][2]), 3)],
                    "type": 1 if nxt else 0,
                    "branch_mode": 2 if len(nxt) > 1 else 0,
                    "next_indices": nxt,
                }
            )
        waypoints = new_wps

    _recompute_yaw(waypoints)
    return waypoints


def _redirect_edges_to_straight_descendant(
    waypoints: list[dict],
    passes: int,
    search_depth: int,
    max_redirect_dist2d: float,
    max_redirect_up_dz: float,
    max_redirect_abs_dz: float,
    max_path_turn_deg: float,
    corridor_width2d: float,
) -> list[dict]:
    for _ in range(max(1, passes)):
        out_adj, _ = _build_graph(waypoints)
        changed = False

        for u in range(len(waypoints)):
            outs = sorted(out_adj.get(u, set()))
            if not outs:
                continue
            pu = waypoints[u]["pos"]
            new_outs = set(outs)

            for v in outs:
                best = v
                best_dist = _dist2d(pu, waypoints[v]["pos"])
                stack: list[tuple[int, int, list[int]]] = [(v, 0, [v])]
                while stack:
                    cur, depth, path_nodes = stack.pop()
                    if depth >= search_depth:
                        continue
                    for nxt in out_adj.get(cur, set()):
                        if nxt == u or nxt in path_nodes:
                            continue
                        path2 = path_nodes + [nxt]
                        pn = waypoints[nxt]["pos"]
                        d2 = _dist2d(pu, pn)
                        if d2 > max_redirect_dist2d:
                            continue
                        if (pn[2] - pu[2]) > max_redirect_up_dz:
                            continue
                        if abs(pn[2] - pu[2]) > max_redirect_abs_dz:
                            continue

                        ok = True
                        prev = pu
                        for mid_i in path2[:-1]:
                            pm = waypoints[mid_i]["pos"]
                            if _point_line_dist2d(pu, pn, pm) > corridor_width2d:
                                ok = False
                                break
                            turn = _turn_deg(prev, pm, pn)
                            if turn > max_path_turn_deg:
                                ok = False
                                break
                            prev = pm
                        if not ok:
                            stack.append((nxt, depth + 1, path2))
                            continue

                        if d2 > best_dist + 10.0:
                            best = nxt
                            best_dist = d2
                        stack.append((nxt, depth + 1, path2))

                if best != v:
                    new_outs.discard(v)
                    new_outs.add(best)
                    changed = True

            out_adj[u] = new_outs

        if not changed:
            break

        new_waypoints: list[dict] = []
        for i, wp in enumerate(waypoints):
            nxt = sorted(out_adj.get(i, set()))
            new_waypoints.append(
                {
                    "pos": [round(float(wp["pos"][0]), 3), round(float(wp["pos"][1]), 3), round(float(wp["pos"][2]), 3)],
                    "angle": [round(float(wp["angle"][0]), 3), round(float(wp["angle"][1]), 3), round(float(wp["angle"][2]), 3)],
                    "type": 1 if nxt else 0,
                    "branch_mode": 2 if len(nxt) > 1 else 0,
                    "next_indices": nxt,
                }
            )
        waypoints = new_waypoints

    _recompute_yaw(waypoints)
    return waypoints


def simplify_paths(
    input_path: Path,
    output_path: Path,
    merge_radius_xy: float,
    merge_radius_z: float,
    collapse_passes: int,
    turn_keep_deg: float,
    max_merge_seg_2d: float,
    max_merge_dz: float,
    backbone_tree: bool,
    diagonal_shortcuts: bool,
    max_shortcut_dist2d: float,
    max_shortcut_up_dz: float,
    max_shortcut_abs_dz: float,
    shortcut_corridor_width2d: float,
    redirect_single_out: bool,
    redirect_passes: int,
    max_redirect_dist2d: float,
    max_redirect_up_dz: float,
    max_redirect_abs_dz: float,
    max_redirect_turn_deg: float,
    centroid_merge_passes: int,
    edge_desc_redirect_passes: int,
    edge_desc_search_depth: int,
    edge_desc_max_turn_deg: float,
    edge_desc_corridor_width2d: float,
    max_walk_up_dz: float,
    max_walk_abs_dz: float,
    max_walk_grade: float,
    min_ramp_dist2d: float,
    hard_max_up_dz: float,
    hard_max_abs_dz: float,
    allow_slope_chain_exception: bool,
    slope_chain_min_dist2d: float,
    slope_chain_max_grade: float,
    slope_chain_max_sign_changes: int,
    slope_chain_edge_up_dz: float,
    slope_chain_edge_abs_dz: float,
    reconnect_slopes: bool,
    slope_min_link_dist2d: float,
    slope_max_link_dist2d: float,
    slope_max_link_up_dz: float,
    slope_max_link_abs_dz: float,
    slope_max_grade: float,
    slope_hard_max_up_dz: float,
    slope_hard_max_abs_dz: float,
    slope_max_new_links_per_node: int,
) -> tuple[int, int]:
    data = json.loads(input_path.read_text(encoding="utf-8"))
    waypoints = data.get("waypoints", [])
    if not isinstance(waypoints, list) or not waypoints:
        raise RuntimeError("paths.json has no waypoints")

    original_count = len(waypoints)
    waypoints = _merge_nearby_centroids(
        waypoints,
        merge_radius_xy=(merge_radius_xy * 0.75),
        merge_radius_z=(merge_radius_z * 0.75),
        passes=centroid_merge_passes,
    )
    waypoints = _merge_close_nodes(waypoints, merge_radius_xy, merge_radius_z)
    waypoints = _collapse_linear_nodes(
        waypoints,
        max_passes=collapse_passes,
        turn_keep_deg=turn_keep_deg,
        max_merge_seg_2d=max_merge_seg_2d,
        max_merge_dz=max_merge_dz,
    )
    if backbone_tree:
        waypoints = _build_backbone_tree(waypoints)
    if diagonal_shortcuts:
        waypoints = _apply_diagonal_shortcuts(
            waypoints,
            max_shortcut_dist2d=max_shortcut_dist2d,
            max_shortcut_up_dz=max_shortcut_up_dz,
            max_shortcut_abs_dz=max_shortcut_abs_dz,
            corridor_width2d=shortcut_corridor_width2d,
        )
    waypoints = _redirect_edges_to_straight_descendant(
        waypoints,
        passes=edge_desc_redirect_passes,
        search_depth=edge_desc_search_depth,
        max_redirect_dist2d=max_redirect_dist2d,
        max_redirect_up_dz=max_redirect_up_dz,
        max_redirect_abs_dz=max_redirect_abs_dz,
        max_path_turn_deg=edge_desc_max_turn_deg,
        corridor_width2d=edge_desc_corridor_width2d,
    )
    if redirect_single_out:
        waypoints = _redirect_single_out_straight(
            waypoints,
            max_passes=redirect_passes,
            max_redirect_dist2d=max_redirect_dist2d,
            max_redirect_up_dz=max_redirect_up_dz,
            max_redirect_abs_dz=max_redirect_abs_dz,
            max_redirect_turn_deg=max_redirect_turn_deg,
        )
    # Final centroid merge to remove newly created near-duplicates.
    waypoints = _merge_nearby_centroids(
        waypoints,
        merge_radius_xy=(merge_radius_xy * 0.55),
        merge_radius_z=(merge_radius_z * 0.55),
        passes=1,
    )
    waypoints = _filter_unwalkable_jump_edges(
        waypoints,
        max_walk_up_dz=max_walk_up_dz,
        max_walk_abs_dz=max_walk_abs_dz,
        max_walk_grade=max_walk_grade,
        min_ramp_dist2d=min_ramp_dist2d,
        hard_max_up_dz=hard_max_up_dz,
        hard_max_abs_dz=hard_max_abs_dz,
        allow_slope_chain_exception=allow_slope_chain_exception,
        slope_chain_min_dist2d=slope_chain_min_dist2d,
        slope_chain_max_grade=slope_chain_max_grade,
        slope_chain_max_sign_changes=slope_chain_max_sign_changes,
        slope_chain_edge_up_dz=slope_chain_edge_up_dz,
        slope_chain_edge_abs_dz=slope_chain_edge_abs_dz,
    )
    if reconnect_slopes:
        waypoints = _reconnect_walkable_slopes(
            waypoints,
            min_link_dist2d=slope_min_link_dist2d,
            max_link_dist2d=slope_max_link_dist2d,
            max_link_up_dz=slope_max_link_up_dz,
            max_link_abs_dz=slope_max_link_abs_dz,
            max_grade=slope_max_grade,
            hard_max_up_dz=slope_hard_max_up_dz,
            hard_max_abs_dz=slope_hard_max_abs_dz,
            max_new_links_per_node=slope_max_new_links_per_node,
        )
    _recompute_yaw(waypoints)

    output = {"waypoints": waypoints}
    output_path.write_text(json.dumps(output, ensure_ascii=False, indent=2), encoding="utf-8")
    return original_count, len(waypoints)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Simplify CS2 paths.json into straighter centerline-like routes while keeping broad map coverage."
    )
    parser.add_argument("--input", default="CS2_WalkBot_Ext/paths.json")
    parser.add_argument("--output", default="CS2_WalkBot_Ext/paths.json")
    parser.add_argument("--merge-radius-xy", type=float, default=70.0)
    parser.add_argument("--merge-radius-z", type=float, default=56.0)
    parser.add_argument("--collapse-passes", type=int, default=16)
    parser.add_argument("--turn-keep-deg", type=float, default=30.0)
    parser.add_argument("--max-merge-seg-2d", type=float, default=360.0)
    parser.add_argument("--max-merge-dz", type=float, default=80.0)
    parser.add_argument(
        "--no-backbone-tree",
        action="store_true",
        help="Disable tree pruning (keep denser graph)",
    )
    parser.add_argument(
        "--no-diagonal-shortcuts",
        action="store_true",
        help="Disable diagonal straight-line shortcuts on linear chains",
    )
    parser.add_argument("--max-shortcut-dist2d", type=float, default=520.0)
    parser.add_argument("--max-shortcut-up-dz", type=float, default=18.0)
    parser.add_argument("--max-shortcut-abs-dz", type=float, default=68.0)
    parser.add_argument("--shortcut-corridor-width2d", type=float, default=36.0)
    parser.add_argument(
        "--no-redirect-single-out",
        action="store_true",
        help="Disable single-outgoing edge redirection to straighter successor",
    )
    parser.add_argument("--redirect-passes", type=int, default=3)
    parser.add_argument("--max-redirect-dist2d", type=float, default=620.0)
    parser.add_argument("--max-redirect-up-dz", type=float, default=18.0)
    parser.add_argument("--max-redirect-abs-dz", type=float, default=70.0)
    parser.add_argument("--max-redirect-turn-deg", type=float, default=38.0)
    parser.add_argument("--centroid-merge-passes", type=int, default=2)
    parser.add_argument("--edge-desc-redirect-passes", type=int, default=2)
    parser.add_argument("--edge-desc-search-depth", type=int, default=3)
    parser.add_argument("--edge-desc-max-turn-deg", type=float, default=44.0)
    parser.add_argument("--edge-desc-corridor-width2d", type=float, default=42.0)
    parser.add_argument(
        "--max-walk-up-dz",
        type=float,
        default=18.0,
        help="Drop edges that require upward step higher than this (jump-like).",
    )
    parser.add_argument(
        "--max-walk-abs-dz",
        type=float,
        default=74.0,
        help="Drop edges with too large absolute Z step.",
    )
    parser.add_argument(
        "--max-walk-grade",
        type=float,
        default=0.42,
        help="Allow ramp-like edges when |dz|/dist2d <= this grade.",
    )
    parser.add_argument(
        "--min-ramp-dist2d",
        type=float,
        default=90.0,
        help="Ramp exception only applies when XY segment length >= this.",
    )
    parser.add_argument(
        "--hard-max-up-dz",
        type=float,
        default=96.0,
        help="Absolute hard cap for upward dz even on ramps.",
    )
    parser.add_argument(
        "--hard-max-abs-dz",
        type=float,
        default=140.0,
        help="Absolute hard cap for |dz| even on ramps.",
    )
    parser.add_argument(
        "--no-slope-chain-exception",
        action="store_true",
        help="Disable slope-chain detection when filtering jump-like edges.",
    )
    parser.add_argument("--slope-chain-min-dist2d", type=float, default=160.0)
    parser.add_argument("--slope-chain-max-grade", type=float, default=0.40)
    parser.add_argument("--slope-chain-max-sign-changes", type=int, default=1)
    parser.add_argument("--slope-chain-edge-up-dz", type=float, default=28.0)
    parser.add_argument("--slope-chain-edge-abs-dz", type=float, default=88.0)
    parser.add_argument(
        "--no-reconnect-slopes",
        action="store_true",
        help="Disable reconnecting nearby walkable slope components.",
    )
    parser.add_argument("--slope-min-link-dist2d", type=float, default=36.0)
    parser.add_argument("--slope-max-link-dist2d", type=float, default=240.0)
    parser.add_argument("--slope-max-link-up-dz", type=float, default=20.0)
    parser.add_argument("--slope-max-link-abs-dz", type=float, default=86.0)
    parser.add_argument("--slope-max-grade", type=float, default=0.40)
    parser.add_argument("--slope-hard-max-up-dz", type=float, default=96.0)
    parser.add_argument("--slope-hard-max-abs-dz", type=float, default=140.0)
    parser.add_argument("--slope-max-new-links-per-node", type=int, default=2)
    args = parser.parse_args()

    old_n, new_n = simplify_paths(
        input_path=Path(args.input),
        output_path=Path(args.output),
        merge_radius_xy=args.merge_radius_xy,
        merge_radius_z=args.merge_radius_z,
        collapse_passes=args.collapse_passes,
        turn_keep_deg=args.turn_keep_deg,
        max_merge_seg_2d=args.max_merge_seg_2d,
        max_merge_dz=args.max_merge_dz,
        backbone_tree=(not args.no_backbone_tree),
        diagonal_shortcuts=(not args.no_diagonal_shortcuts),
        max_shortcut_dist2d=args.max_shortcut_dist2d,
        max_shortcut_up_dz=args.max_shortcut_up_dz,
        max_shortcut_abs_dz=args.max_shortcut_abs_dz,
        shortcut_corridor_width2d=args.shortcut_corridor_width2d,
        redirect_single_out=(not args.no_redirect_single_out),
        redirect_passes=args.redirect_passes,
        max_redirect_dist2d=args.max_redirect_dist2d,
        max_redirect_up_dz=args.max_redirect_up_dz,
        max_redirect_abs_dz=args.max_redirect_abs_dz,
        max_redirect_turn_deg=args.max_redirect_turn_deg,
        centroid_merge_passes=args.centroid_merge_passes,
        edge_desc_redirect_passes=args.edge_desc_redirect_passes,
        edge_desc_search_depth=args.edge_desc_search_depth,
        edge_desc_max_turn_deg=args.edge_desc_max_turn_deg,
        edge_desc_corridor_width2d=args.edge_desc_corridor_width2d,
        max_walk_up_dz=args.max_walk_up_dz,
        max_walk_abs_dz=args.max_walk_abs_dz,
        max_walk_grade=args.max_walk_grade,
        min_ramp_dist2d=args.min_ramp_dist2d,
        hard_max_up_dz=args.hard_max_up_dz,
        hard_max_abs_dz=args.hard_max_abs_dz,
        allow_slope_chain_exception=(not args.no_slope_chain_exception),
        slope_chain_min_dist2d=args.slope_chain_min_dist2d,
        slope_chain_max_grade=args.slope_chain_max_grade,
        slope_chain_max_sign_changes=args.slope_chain_max_sign_changes,
        slope_chain_edge_up_dz=args.slope_chain_edge_up_dz,
        slope_chain_edge_abs_dz=args.slope_chain_edge_abs_dz,
        reconnect_slopes=(not args.no_reconnect_slopes),
        slope_min_link_dist2d=args.slope_min_link_dist2d,
        slope_max_link_dist2d=args.slope_max_link_dist2d,
        slope_max_link_up_dz=args.slope_max_link_up_dz,
        slope_max_link_abs_dz=args.slope_max_link_abs_dz,
        slope_max_grade=args.slope_max_grade,
        slope_hard_max_up_dz=args.slope_hard_max_up_dz,
        slope_hard_max_abs_dz=args.slope_hard_max_abs_dz,
        slope_max_new_links_per_node=args.slope_max_new_links_per_node,
    )
    print(f"[+] Simplified waypoints: {old_n} -> {new_n}")
    print(f"[+] Wrote: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
