/* Copyright © 2017 Cormac O'Brien
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Stack;

public class EightPuzzle {
    enum AStarHeuristic {
        MISPLACED, MANHATTAN
    }

    enum Direction {
        UP,
        DOWN,
        LEFT,
        RIGHT;

        public String toString() {
            switch (this) {
                case UP:
                    return "up";
                case DOWN:
                    return "down";
                case LEFT:
                    return "left";
                case RIGHT:
                    return "right";
                default:
                    throw new RuntimeException("Invalid direction value");
            }
        }
    }

    /*
     * A transition between two valid 8-puzzle states.
     */
    static class Move {
        Direction dir;
        Node node;
        int cost;

        Move(Direction dir, Node node, int cost) {
            this.dir = dir;
            this.node = node;
            this.cost = cost;
        }

        Direction getDirection() {
            return this.dir;
        }

        Node getNode() {
            return this.node;
        }

        int getCost() {
            return this.cost;
        }
    }

    /*
     * Search node for the 8-puzzle problem.
     *
     * Represents the grid internally as a 9-element byte array. The empty space
     * is represented by 0. Equality and hashing are implemented with
     * Arrays.equals() and Arrays.hashCode().
     *
     * Neighbors are calculated by interpreting the grid as a 2-D array and, if
     * possible, swapping 0 with the elements above, below, left and right of
     * it. The neighboring nodes are returned as a List<Node, Integer> which
     * associates each neighbor with the cost of the edge to it.
     */
    static class Node {
        byte[] grid;

        Node(byte[] grid) {
            if (grid.length != 9) {
                throw new IllegalArgumentException();
            }

            boolean[] present = new boolean[9];

            for (int i = 0; i < 9; i++) {
                if (present[grid[i]]) {
                    throw new IllegalArgumentException("duplicate tile in grid");
                }

                present[grid[i]] = true;
            }

            for (int i = 0; i < 9; i++) {
                if (!present[i]) {
                    throw new IllegalArgumentException("missing tile in grid");
                }
            }

            this.grid = grid;
        }

        @Override
        public boolean equals(Object other) {
            if (!(other instanceof Node)) {
                return false;
            }

            if (other == this) {
                return true;
            }

            Node node = (Node) other;
            return Arrays.equals(this.grid, node.grid);
        }

        @Override
        public int hashCode() {
            return Arrays.hashCode(this.grid);
        }

        public List<Move> neighbors() {
            int zero = -1;

            for (int i = 0; i < this.grid.length; i++) {
                if (this.grid[i] == 0) {
                    zero = i;
                    break;
                }
            }

            if (zero == -1) {
                throw new RuntimeException();
            }

            List<Move> neighbors = new ArrayList<>();

            // empty space not in top row
            if (zero > 2) {
                byte[] new_grid = Arrays.copyOf(this.grid, this.grid.length);
                new_grid[zero] = new_grid[zero - 3];
                new_grid[zero - 3] = 0;
                neighbors.add(new Move(Direction.UP, new Node(new_grid),
                        new Integer((int) new_grid[zero])));
            }

            // empty space not in bottom row
            if (zero < 6) {
                byte[] new_grid = Arrays.copyOf(this.grid, this.grid.length);
                new_grid[zero] = new_grid[zero + 3];
                new_grid[zero + 3] = 0;
                neighbors.add(new Move(Direction.DOWN, new Node(new_grid),
                        new Integer((int) new_grid[zero])));
            }

            // empty space not in left column
            if (zero % 3 > 0) {
                byte[] new_grid = Arrays.copyOf(this.grid, this.grid.length);
                new_grid[zero] = new_grid[zero - 1];
                new_grid[zero - 1] = 0;
                neighbors.add(new Move(Direction.LEFT, new Node(new_grid),
                        new Integer((int) new_grid[zero])));
            }

            // empty space not in right column
            if (zero % 3 < 2) {
                byte[] new_grid = Arrays.copyOf(this.grid, this.grid.length);
                new_grid[zero] = new_grid[zero + 1];
                new_grid[zero + 1] = 0;
                neighbors.add(new Move(Direction.RIGHT, new Node(new_grid),
                        new Integer((int) new_grid[zero])));
            }

            return neighbors;
        }

        public int misplaced() {
            int result = 0;

            for (int i = 0; i < this.grid.length; i++) {
                if (this.grid[i] != EASY.grid[i]) {
                    result += 1;
                }
            }

            return result;
        }

        public int manhattan() {
            int result = 0;

            int[] grid_rs = new int[9];
            int[] grid_cs = new int[9];
            int[] goal_rs = new int[9];
            int[] goal_cs = new int[9];

            for (int r = 0; r < 3; r++) {
                for (int c = 0; c < 3; c++) {
                    grid_rs[this.grid[3 * r + c]] = r;
                    grid_cs[this.grid[3 * r + c]] = c;
                    goal_rs[GOAL.grid[3 * r + c]] = r;
                    goal_cs[GOAL.grid[3 * r + c]] = c;
                }
            }

            for (int i = 1; i < 9; i++) {
                result += Math.abs(grid_rs[i] - goal_rs[i]) + Math.abs(grid_cs[i] - goal_cs[i]);
            }

            return result;
        }

        public int heuristic() {
            if (aStarHeuristic == AStarHeuristic.MANHATTAN) {
                return this.manhattan();
            } else if (aStarHeuristic == AStarHeuristic.MISPLACED) {
                return this.misplaced();
            } else {
                throw new RuntimeException("No heuristic selected");
            }
        }

        public String toString() {
            StringBuffer buf = new StringBuffer();
            for (int i = 0; i < 9; i++) {
                buf.append(String.format("%d ", (int) this.grid[i]));
            }
            return buf.toString();
        }
    }

    static class SearchResult {
        List<Move> path;
        int cost;
        int time;
        int space;

        SearchResult(List<Move> path, int cost, int time, int space) {
            this.path = path;
            this.cost = cost;
            this.time = time;
            this.space = space;
        }

        public void print() {
            System.out.printf("SEARCH RESULTS\n");
            System.out.printf("Length | %d moves\n", this.path.size());
            System.out.printf("Cost   | %d\n", this.cost);
            System.out.printf("Time   | %d pops\n", this.time);
            System.out.printf("Space  | %d nodes\n", this.space);
        }

        public void printPath() {
            for (Move m: this.path) {
                System.out.printf("%s-> %s (%d)\n", m.getNode().toString(), m.getDirection().toString(), m.getCost());
            }
            System.out.println(GOAL.toString());
        }
    }

    public static AStarHeuristic aStarHeuristic = null;

    public static final Node GOAL = new Node(new byte[] { 1, 2, 3, 8, 0, 4, 7, 6, 5 });
    public static final Node EASY = new Node(new byte[] { 1, 3, 4, 8, 6, 2, 7, 0, 5, });
    public static final Node MEDIUM = new Node(new byte[] { 2, 8, 1, 0, 4, 3, 7, 6, 5, });
    public static final Node HARD = new Node(new byte[] { 5, 6, 7, 4, 0, 8, 3, 2, 1, });

    public static SearchResult searchResult(HashMap<Node, Move> from, int time, int space) {
        int cost = 0;
        List<Move> path = new ArrayList<>();
        for (Move current = from.get(GOAL); current != null; current = from.get(current.getNode())) {
            path.add(current);
            cost += current.getCost();
        }
        Collections.reverse(path);

        return new SearchResult(path, cost, time, space);
    }

    /*
     * ==================================== SEARCH ALGORITHMS ====================================
     *
     * The search algorithms are all implemented in a similar way.
     * - open: the open (frontier) set. These nodes have been encountered but not yet expanded.
     * - closed: the closed set. These nodes have been expanded.
     * - from: the previous node in the path.
     * - cost: the esitmated cost of pathing to or through this node.
     *
     * Additionally, some of the implementations use a variable open_set, which acts as a fast
     * lookup table for determining whether the actual open set contains a value or not. This is
     * O(1) in the average case vs. O(n) searching through the priority queue, which cuts the
     * uniform-cost and A* implementations from 30 seconds on hard to 1 second.
     */

    /*
     * Breadth-first search.
     */
    public static SearchResult bfs(Node start) {
        HashSet<Node> closed = new HashSet<>();
        HashMap<Node, Move> from = new HashMap<>();

        /*
         * BFS uses a Queue
         */
        ArrayDeque<Node> open = new ArrayDeque<>();

        int space = 0;
        open.addLast(start);

        while (!open.isEmpty()) {
            Node current = open.remove();
            closed.add(current);

            if (current.equals(GOAL)) {
                return searchResult(from, closed.size(), space);
            }

            for (Move m : current.neighbors()) {
                Node n = m.getNode();

                if (closed.contains(n)) {
                    continue;
                }

                from.put(n, new Move(m.getDirection(), current, m.getCost()));
                open.addLast(n);
            }

            if (open.size() > space) {
                space = open.size();
            }
        }

        throw new RuntimeException("No solution found");
    }

    /*
     * Depth-first search.
     */
    public static SearchResult dfs(Node start) {
        /*
         * DFS uses a stack
         */
        Stack<Node> open = new Stack<>();
        HashSet<Node> closed = new HashSet<>();
        HashMap<Node, Move> from = new HashMap<>();

        int space = 0;
        open.push(start);

        while (!open.isEmpty()) {
            Node current = open.pop();
            closed.add(current);

            if (current.equals(GOAL)) {
                return searchResult(from, closed.size(), space);
            }

            for (Move m : current.neighbors()) {
                Node n = m.getNode();

                if (closed.contains(n)) {
                    continue;
                }

                from.put(n, new Move(m.getDirection(), current, m.getCost()));
                open.push(n);
            }

            if (space < open.size()) {
                space = open.size();
            }
        }

        throw new RuntimeException("No solution found");
    }

    /*
     * Uniform-cost search.
     */
    public static SearchResult uniformCost(Node start) {
        HashSet<Node> closed = new HashSet<>();
        HashMap<Node, Integer> cost = new HashMap<>();
        HashMap<Node, Move> from = new HashMap<>();

        PriorityQueue<Node> open = new PriorityQueue<>(128, new Comparator<Node>() {
            public int compare(Node n1, Node n2) {
                return cost.getOrDefault(n1, Integer.MIN_VALUE)
                        - cost.getOrDefault(n2, Integer.MIN_VALUE);
            }
        });
        HashSet<Node> open_set = new HashSet<>();

        int space = 0;
        open.add(start);
        open_set.add(start);
        cost.put(start, 0);

        while (!open.isEmpty()) {
            Node current = open.remove();
            open_set.remove(current);
            closed.add(current);

            if (current.equals(GOAL)) {
                return searchResult(from, closed.size(), space);
            }

            for (Move m : current.neighbors()) {
                Node n = m.getNode();
                Integer c = m.getCost();

                if (closed.contains(n)) {
                    continue;
                }

                int new_cost = cost.get(current) + c;

                if (!open_set.contains(n)) {
                    open.add(n);
                    open_set.add(n);
                } else if (new_cost >= cost.getOrDefault(n, Integer.MAX_VALUE)) {
                    continue;
                }

                from.put(n, new Move(m.getDirection(), current, m.getCost()));
                cost.put(n, new_cost);
            }

            if (space < open.size()) {
                space = open.size();
            }
        }

        throw new RuntimeException("No solution found");
    }

    /*
     * Greedy best-first search.
     */
    public static SearchResult bestFirst(Node start) {
        HashSet<Node> closed = new HashSet<>();
        HashMap<Node, Move> from = new HashMap<>();
        PriorityQueue<Node> open = new PriorityQueue<>(128, new Comparator<Node>() {
                public int compare(Node n1, Node n2) {
                    return n1.misplaced() - n2.misplaced();
                }
            });

        int space = 0;
        open.add(start);

        while (!open.isEmpty()) {
            Node current = open.remove();
            closed.add(current);

            if (current.equals(GOAL)) {
                return searchResult(from, closed.size(), space);
            }

            for (Move m: current.neighbors()) {
                Node n = m.getNode();

                if (closed.contains(n)) {
                    continue;
                }

                if (!open.contains(n)) {
                    open.add(n);
                }

                from.put(n, new Move(m.getDirection(), current, m.getCost()));
            }

            if (space < open.size()) {
                space = open.size();
            }
        }
        throw new RuntimeException("No solution found");
    }

    /*
     * A* search.
     */
    public static SearchResult aStar(Node start) {
        HashSet<Node> closed = new HashSet<>();
        HashMap<Node, Move> from = new HashMap<>();
        HashMap<Node, Integer> partial_cost = new HashMap<>();
        HashMap<Node, Integer> total_cost = new HashMap<>();
        PriorityQueue<Node> open = new PriorityQueue<>(128, new Comparator<Node>() {
            public int compare(Node n1, Node n2) {
                return total_cost.getOrDefault(n1, Integer.MIN_VALUE)
                        - total_cost.getOrDefault(n2, Integer.MIN_VALUE);
            }
        });
        HashSet<Node> open_set = new HashSet<>();

        int space = 0;
        open.add(start);
        open_set.add(start);
        partial_cost.put(start, 0);
        total_cost.put(start, start.heuristic());

        while (!open.isEmpty()) {
            Node current = open.remove();
            open_set.remove(current);
            closed.add(current);

            if (current.equals(GOAL)) {
                return searchResult(from, closed.size(), space);
            }

            for (Move m : current.neighbors()) {
                Node n = m.getNode();
                Integer c = m.getCost();

                if (closed.contains(n)) {
                    continue;
                }

                /*
                 * Recalculate cost
                 */
                int partial = partial_cost.get(current) + c;

                if (!(open_set.contains(n))) {
                    open.add(n);
                    open_set.add(n);
                } else if (!(partial < partial_cost.getOrDefault(n, Integer.MAX_VALUE))) {
                    continue;
                }

                /*
                 * If new cost is lower, update the graph
                 */
                from.put(n, new Move(m.getDirection(), current, m.getCost()));
                partial_cost.put(n, partial);
                total_cost.put(n, partial + n.heuristic());
            }

            if (space < open.size()) {
                space = open.size();
            }
        }

        throw new RuntimeException("No solution found.");
    }

    static final String USAGE = "usage: java Search <METHOD> <DIFFICULTY>\n\n"
            + "method must be one of:\n" + " bfs (breadth-first)\n" + " dfs (depth-first)\n"
            + " uniform-cost\n" + " best-first\n"
            + " astar1 (A* where h = number of misplaced tiles)\n"
            + " astar2 (A* where h = cumulative manhattan distance of all tiles from goal positions)\n\n"
            + "difficulty must be one of:\n" + " easy\n" + " medium\n" + " hard\n";

    public static void usage() {
        System.out.println(USAGE);
    }

    public static void main(String[] args) {
        if (args.length == 1 && args[0].equals("--help")) {
            usage();
        } else if (args.length == 2) {
            Node start = null;
            switch (args[1]) {
            case "easy":
                start = EASY;
                break;
            case "medium":
                start = MEDIUM;
                break;
            case "hard":
                start = HARD;
                break;
            default:
                System.out.printf("\"%s\" is not a recognized difficulty\n", args[1]);
                System.exit(1);
            }

            SearchResult result = null;
            switch (args[0]) {
            case "bfs":
                result = bfs(start);
                break;
            case "dfs":
                result = dfs(start);
                break;
            case "uniform-cost":
                result = uniformCost(start);
                break;
            case "best-first":
                result = bestFirst(start);
                break;
            case "astar1":
                aStarHeuristic = AStarHeuristic.MISPLACED;
                result = aStar(start);
                break;
            case "astar2":
                aStarHeuristic = AStarHeuristic.MANHATTAN;
                result = aStar(start);
                break;
            default:
                System.out.printf("\"%s\" is not a recognized search method\n", args[0]);
                System.exit(1);
            }

            result.print();
            result.printPath();
        } else {
            usage();
        }
    }
}
