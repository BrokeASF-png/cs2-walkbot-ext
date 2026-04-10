#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
from collections import defaultdict
from pathlib import Path

from demoparser2 import DemoParser


def _is_finite(value: float) -> bool:
    return math.isfinite(value)


def _yaw_deg(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.degrees(math.atan2(b[1] - a[1], b[0] - a[0]))


def _build_play_mask(df) -> "object":
    return (
        (df["is_alive"] == True)
        & (df["is_match_started"] == True)
        & (df["is_freeze_period"] == False)
        & (df["is_warmup_period"] == False)
        & (df["is_terrorist_timeout"] == False)
        & (df["is_ct_timeout"] == False)
        & (df["is_technical_timeout"] == False)
        & (df["is_waiting_for_resume"] == False)
        & (df["game_phase"].isin([2, 3]))
        & (df["team_name"].isin(["CT", "TERRORIST"]))
    )


def generate_paths(
    demo_path: Path,
    output_path: Path,
    grid_xy: float,
    grid_z: float,
    min_node_visits: int,
    min_edge_visits: int,
    max_edge_dist: float,
    max_edge_dz: float,
    jump_up_limit: float,
    simplify_turn_deg: float,
    simplify_dist_limit: float,
    simplify_passes: int,
    require_direct_edge_for_collapse: bool,
    keep_largest_component_only: bool,
    min_component_nodes: int,
) -> tuple[int, int]:
    parser = DemoParser(str(demo_path))
    wanted_props = [
        "X",
        "Y",
        "Z",
        "tick",
        "steamid",
        "team_name",
        "is_alive",
        "is_freeze_period",
        "is_warmup_period",
        "is_terrorist_timeout",
        "is_ct_timeout",
        "is_technical_timeout",
        "is_waiting_for_resume",
        "is_match_started",
        "game_phase",
    ]
    df = parser.parse_ticks(wanted_props=wanted_props)
    df = df[_build_play_mask(df)].copy()

    # Remove invalid coordinates.
    df = df[
        df["X"].map(_is_finite) & df["Y"].map(_is_finite) & df["Z"].map(_is_finite)
    ]
    df = df[(df["X"].abs() + df["Y"].abs()) > 1.0]
    df = df.sort_values(["steamid", "tick"])

    if df.empty:
        raise RuntimeError("No valid in-play movement ticks found in demo.")

    def to_cell(x: float, y: float, z: float) -> tuple[int, int, int]:
        return (
            int(round(x / grid_xy)),
            int(round(y / grid_xy)),
            int(round(z / grid_z)),
        )

    node_visits: dict[tuple[int, int, int], int] = defaultdict(int)
    node_sum: dict[tuple[int, int, int], list[float]] = defaultdict(
        lambda: [0.0, 0.0, 0.0]
    )
    edge_counts: dict[tuple[tuple[int, int, int], tuple[int, int, int]], int] = defaultdict(
        int
    )

    for _, player in df.groupby("steamid", sort=False):
        keys: list[tuple[int, int, int]] = []
        pos: list[tuple[float, float, float]] = []
        for x, y, z in zip(player["X"].values, player["Y"].values, player["Z"].values):
            key = to_cell(float(x), float(y), float(z))
            if keys and key == keys[-1]:
                continue
            keys.append(key)
            pos.append((float(x), float(y), float(z)))

        if not keys:
            continue

        for key, (x, y, z) in zip(keys, pos):
            node_visits[key] += 1
            acc = node_sum[key]
            acc[0] += x
            acc[1] += y
            acc[2] += z

        for i in range(len(keys) - 1):
            a = keys[i]
            b = keys[i + 1]
            if a == b:
                continue
            ax, ay, az = pos[i]
            bx, by, bz = pos[i + 1]
            dist = math.dist((ax, ay), (bx, by))
            dz = abs(bz - az)
            if dist > max_edge_dist or dz > max_edge_dz:
                continue
            edge_counts[(a, b)] += 1

    kept_nodes = {k for k, c in node_visits.items() if c >= min_node_visits}
    kept_edges = {
        (a, b): c
        for (a, b), c in edge_counts.items()
        if c >= min_edge_visits and a in kept_nodes and b in kept_nodes
    }

    # Keep only nodes that are part of kept edges.
    edge_nodes = set()
    for a, b in kept_edges:
        edge_nodes.add(a)
        edge_nodes.add(b)
    kept_nodes = kept_nodes & edge_nodes
    if not kept_nodes:
        raise RuntimeError("No nodes survived filtering; lower filtering thresholds.")

    centers: dict[tuple[int, int, int], tuple[float, float, float]] = {}
    for key in kept_nodes:
        sx, sy, sz = node_sum[key]
        n = float(node_visits[key])
        centers[key] = (sx / n, sy / n, sz / n)

    # Remove "jump-up" edges (upward dz too large), keep normal floor/stair transitions.
    kept_edges = {
        (a, b): c
        for (a, b), c in kept_edges.items()
        if (centers[b][2] - centers[a][2]) <= jump_up_limit
    }
    if not kept_edges:
        raise RuntimeError("No edges after jump-up filtering; increase --jump-up-limit.")

    # Rebuild nodes from filtered edges.
    edge_nodes = set()
    for a, b in kept_edges:
        edge_nodes.add(a)
        edge_nodes.add(b)
    kept_nodes = kept_nodes & edge_nodes

    undirected = defaultdict(set)
    for a, b in kept_edges:
        undirected[a].add(b)
        undirected[b].add(a)

    # Keep either only largest component or all components above a size threshold.
    visited = set()
    components: list[set[tuple[int, int, int]]] = []
    for node in kept_nodes:
        if node in visited:
            continue
        stack = [node]
        comp = set()
        visited.add(node)
        while stack:
            cur = stack.pop()
            comp.add(cur)
            for nxt in undirected[cur]:
                if nxt not in visited:
                    visited.add(nxt)
                    stack.append(nxt)
        components.append(comp)

    if keep_largest_component_only:
        kept_nodes = max(components, key=len) if components else set()
    else:
        kept_nodes = set()
        for comp in components:
            if len(comp) >= min_component_nodes:
                kept_nodes |= comp

    kept_edges = {
        (a, b): c
        for (a, b), c in kept_edges.items()
        if a in kept_nodes and b in kept_nodes
    }
    observed_edges_after_filter = dict(kept_edges)

    # Graph simplification: collapse linear points on near-straight segments.
    turn_threshold_cos = math.cos(math.radians(simplify_turn_deg))
    for _ in range(max(0, simplify_passes)):
        incoming: dict[tuple[int, int, int], list[tuple[int, int, int]]] = defaultdict(list)
        outgoing_nodes: dict[tuple[int, int, int], list[tuple[int, int, int]]] = defaultdict(list)
        for a, b in kept_edges:
            outgoing_nodes[a].append(b)
            incoming[b].append(a)

        removed_any = False
        for n in list(kept_nodes):
            ins = sorted(set(incoming.get(n, [])))
            outs = sorted(set(outgoing_nodes.get(n, [])))
            if len(ins) != 1 or len(outs) != 1:
                continue
            p = ins[0]
            s = outs[0]
            if p == n or s == n or p == s:
                continue
            if p not in kept_nodes or s not in kept_nodes:
                continue

            pp = centers[p]
            nn = centers[n]
            ss = centers[s]
            v1 = (nn[0] - pp[0], nn[1] - pp[1])
            v2 = (ss[0] - nn[0], ss[1] - nn[1])
            l1 = math.hypot(v1[0], v1[1])
            l2 = math.hypot(v2[0], v2[1])
            if l1 < 1e-3 or l2 < 1e-3:
                continue
            if l1 > simplify_dist_limit or l2 > simplify_dist_limit:
                continue

            # Keep corners, drop mostly straight pass-through points.
            dot = (v1[0] * v2[0] + v1[1] * v2[1]) / (l1 * l2)
            if dot < turn_threshold_cos:
                continue

            # Skip if merged edge would require jump-up.
            if (ss[2] - pp[2]) > jump_up_limit:
                continue

            # Prevent synthetic "shortcut" edges unless directly observed in raw data.
            if require_direct_edge_for_collapse and (p, s) not in observed_edges_after_filter:
                continue

            w_pn = kept_edges.get((p, n), 0)
            w_ns = kept_edges.get((n, s), 0)
            if w_pn <= 0 or w_ns <= 0:
                continue

            # Remove p->n and n->s, insert/merge p->s.
            del kept_edges[(p, n)]
            del kept_edges[(n, s)]
            kept_edges[(p, s)] = kept_edges.get((p, s), 0) + min(w_pn, w_ns)
            kept_nodes.remove(n)
            removed_any = True

        if not removed_any:
            break

    # Stable ordering by visit frequency then XY position.
    node_list = sorted(
        kept_nodes,
        key=lambda k: (-node_visits[k], centers[k][0], centers[k][1], centers[k][2]),
    )
    index_of = {k: i for i, k in enumerate(node_list)}

    outgoing: dict[int, list[int]] = defaultdict(list)
    yaw_accum: dict[int, tuple[float, float]] = defaultdict(lambda: (0.0, 0.0))

    for (a, b), w in kept_edges.items():
        ia = index_of[a]
        ib = index_of[b]
        outgoing[ia].append(ib)
        ax, ay, _ = centers[a]
        bx, by, _ = centers[b]
        dx = bx - ax
        dy = by - ay
        sx, sy = yaw_accum[ia]
        yaw_accum[ia] = (sx + dx * w, sy + dy * w)

    waypoints = []
    for i, key in enumerate(node_list):
        pos = centers[key]
        next_indices = sorted(set(outgoing.get(i, [])))
        sx, sy = yaw_accum.get(i, (0.0, 0.0))
        if abs(sx) + abs(sy) > 1e-5:
            yaw = math.degrees(math.atan2(sy, sx))
        elif next_indices:
            yaw = _yaw_deg(pos, centers[node_list[next_indices[0]]])
        else:
            yaw = 0.0

        wp = {
            "pos": [round(pos[0], 3), round(pos[1], 3), round(pos[2], 3)],
            "angle": [0.0, round(yaw, 3), 0.0],
            "type": 1,
            "branch_mode": 2 if len(next_indices) > 1 else 0,
            "next_indices": next_indices,
        }
        waypoints.append(wp)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        json.dumps({"waypoints": waypoints}, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    return len(waypoints), len(kept_edges)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Generate CS2 WalkBot paths.json from demo movement trajectories."
    )
    parser.add_argument(
        "--demo",
        default=r"D:\SteamLibrary\steamapps\common\Counter-Strike Global Offensive\game\csgo\g161-n-20250609001253809047950_de_mirage.dem",
        help="Input demo file (.dem)",
    )
    parser.add_argument(
        "--output",
        default=r"CS2_WalkBot_Ext\paths.json",
        help="Output paths.json",
    )
    parser.add_argument("--grid-xy", type=float, default=48.0, help="XY grid size")
    parser.add_argument("--grid-z", type=float, default=54.0, help="Z grid size")
    parser.add_argument(
        "--min-node-visits", type=int, default=4, help="Minimum visits to keep a node"
    )
    parser.add_argument(
        "--min-edge-visits", type=int, default=2, help="Minimum transitions to keep an edge"
    )
    parser.add_argument(
        "--max-edge-dist", type=float, default=260.0, help="Drop too-large per-tick jumps in XY"
    )
    parser.add_argument(
        "--max-edge-dz", type=float, default=140.0, help="Drop too-large per-tick jumps in Z"
    )
    parser.add_argument(
        "--jump-up-limit",
        type=float,
        default=16.0,
        help="Maximum allowed upward dz on a kept edge (jump-like edges are removed)",
    )
    parser.add_argument(
        "--simplify-turn-deg",
        type=float,
        default=14.0,
        help="If local turn is below this angle, middle point can be collapsed",
    )
    parser.add_argument(
        "--simplify-dist-limit",
        type=float,
        default=260.0,
        help="Only collapse short adjacent segments (XY distance limit)",
    )
    parser.add_argument(
        "--simplify-passes",
        type=int,
        default=0,
        help="How many simplification passes to run",
    )
    parser.add_argument(
        "--allow-synthetic-collapse-edges",
        action="store_true",
        help="Allow collapse to create shortcut edges not directly observed in demo",
    )
    parser.add_argument(
        "--keep-largest-component-only",
        action="store_true",
        help="Only keep largest connected component (default keeps all sizable components)",
    )
    parser.add_argument(
        "--min-component-nodes",
        type=int,
        default=4,
        help="When keeping all components, drop tiny isolated components below this size",
    )
    args = parser.parse_args()

    demo_path = Path(args.demo)
    output_path = Path(args.output)
    if not demo_path.exists():
        raise FileNotFoundError(f"Demo not found: {demo_path}")

    n_nodes, n_edges = generate_paths(
        demo_path=demo_path,
        output_path=output_path,
        grid_xy=args.grid_xy,
        grid_z=args.grid_z,
        min_node_visits=args.min_node_visits,
        min_edge_visits=args.min_edge_visits,
        max_edge_dist=args.max_edge_dist,
        max_edge_dz=args.max_edge_dz,
        jump_up_limit=args.jump_up_limit,
        simplify_turn_deg=args.simplify_turn_deg,
        simplify_dist_limit=args.simplify_dist_limit,
        simplify_passes=args.simplify_passes,
        require_direct_edge_for_collapse=(not args.allow_synthetic_collapse_edges),
        keep_largest_component_only=args.keep_largest_component_only,
        min_component_nodes=args.min_component_nodes,
    )
    print(f"[+] Generated {n_nodes} waypoints and {n_edges} directed edges")
    print(f"[+] Wrote: {output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
