#!/usr/bin/env python3

import rospy
import random
import math

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import TeleportAbsolute, SetPen
from std_srvs.srv import Empty


# ============================================================
# MAZE SETTINGS
# ============================================================

SIZE = 9

START = (1, 1)
GOAL = (7, 7)

MIN_X = 1.0
MIN_Y = 1.0

CELL_SIZE = 1.15


class MazeSolver:

    def __init__(self):

        rospy.init_node("autonomous_maze_solver")

        self.cmd_pub = rospy.Publisher(
            "/turtle1/cmd_vel",
            Twist,
            queue_size=10
        )

        self.pose = None

        rospy.Subscriber(
            "/turtle1/pose",
            Pose,
            self.pose_callback
        )

        rospy.sleep(1)


    # ========================================================
    # TURTLE POSE
    # ========================================================

    def pose_callback(self, msg):

        self.pose = msg


    # ========================================================
    # RESET TURTLESIM
    # ========================================================

    def reset_turtlesim(self):

        rospy.wait_for_service("/reset")

        reset = rospy.ServiceProxy(
            "/reset",
            Empty
        )

        reset()

        rospy.sleep(1)

        rospy.loginfo("Turtlesim reset.")


    # ========================================================
    # GRID CELL -> TURTLESIM POSITION
    # ========================================================

    def cell_position(self, cell):

        row, col = cell

        x = MIN_X + col * CELL_SIZE
        y = MIN_Y + row * CELL_SIZE

        return x, y


    # ========================================================
    # GENERATE RANDOM MULTI-PATH MAZE
    # ========================================================

    def create_maze(self):

        # Start with everything as a wall
        maze = [
            [1 for _ in range(SIZE)]
            for _ in range(SIZE)
        ]

        # Open starting cell
        maze[START[0]][START[1]] = 0

        stack = [START]

        directions = [
            (-2, 0),
            (2, 0),
            (0, -2),
            (0, 2)
        ]

        # ----------------------------------------------------
        # Generate basic randomized maze
        # ----------------------------------------------------

        while stack:

            current = stack[-1]

            r, c = current

            shuffled = directions[:]
            random.shuffle(shuffled)

            neighbours = []

            for dr, dc in shuffled:

                nr = r + dr
                nc = c + dc

                if nr <= 0 or nr >= SIZE - 1:
                    continue

                if nc <= 0 or nc >= SIZE - 1:
                    continue

                if maze[nr][nc] == 1:

                    neighbours.append(
                        (nr, nc, dr, dc)
                    )

            if neighbours:

                nr, nc, dr, dc = random.choice(
                    neighbours
                )

                # Remove wall between cells
                maze[
                    r + dr // 2
                ][
                    c + dc // 2
                ] = 0

                # Open new cell
                maze[nr][nc] = 0

                stack.append(
                    (nr, nc)
                )

            else:

                stack.pop()


        # ----------------------------------------------------
        # CREATE EXTRA OPENINGS
        #
        # These create loops and alternative routes.
        # ----------------------------------------------------

        possible_openings = []

        for r in range(1, SIZE - 1):

            for c in range(1, SIZE - 1):

                # Horizontal opening
                if c < SIZE - 2:

                    if (
                        maze[r][c] == 0
                        and maze[r][c + 2] == 0
                        and maze[r][c + 1] == 1
                    ):

                        possible_openings.append(
                            (r, c + 1)
                        )

                # Vertical opening
                if r < SIZE - 2:

                    if (
                        maze[r][c] == 0
                        and maze[r + 2][c] == 0
                        and maze[r + 1][c] == 1
                    ):

                        possible_openings.append(
                            (r + 1, c)
                        )


        random.shuffle(
            possible_openings
        )

        # Number of additional openings
        openings_to_create = min(
            5,
            len(possible_openings)
        )

        for i in range(openings_to_create):

            r, c = possible_openings[i]

            maze[r][c] = 0


        # ----------------------------------------------------
        # GUARANTEE START
        # ----------------------------------------------------

        maze[START[0]][START[1]] = 0


        # ----------------------------------------------------
        # GUARANTEE GOAL
        # ----------------------------------------------------

        maze[GOAL[0]][GOAL[1]] = 0


        # Cell immediately before goal
        maze[
            GOAL[0]
        ][
            GOAL[1] - 1
        ] = 0


        rospy.loginfo(
            "Generated multi-path maze."
        )

        rospy.loginfo(
            "Extra openings created: %d",
            openings_to_create
        )

        return maze


    # ========================================================
    # BFS SHORTEST PATH
    # ========================================================

    def solve_maze(self, maze):

        queue = [START]

        visited = {START}

        parent = {}

        directions = [
            (-1, 0),
            (1, 0),
            (0, -1),
            (0, 1)
        ]

        while queue:

            current = queue.pop(0)

            if current == GOAL:

                break

            r, c = current

            for dr, dc in directions:

                nr = r + dr
                nc = c + dc

                # Outside grid
                if nr < 0 or nr >= SIZE:
                    continue

                if nc < 0 or nc >= SIZE:
                    continue

                # Wall
                if maze[nr][nc] == 1:
                    continue

                neighbour = (nr, nc)

                # Already visited
                if neighbour in visited:
                    continue

                visited.add(neighbour)

                parent[neighbour] = current

                queue.append(neighbour)


        # No solution
        if GOAL not in visited:

            return None


        # Reconstruct shortest path
        path = []

        current = GOAL

        while current != START:

            path.append(current)

            current = parent[current]

        path.append(START)

        path.reverse()

        return path


    # ========================================================
    # DRAW MAZE
    # ========================================================

    def draw_maze(self, maze):

        rospy.loginfo(
            "Drawing generated maze..."
        )

        rospy.wait_for_service(
            "/turtle1/teleport_absolute"
        )

        teleport = rospy.ServiceProxy(
            "/turtle1/teleport_absolute",
            TeleportAbsolute
        )

        rospy.wait_for_service(
            "/turtle1/set_pen"
        )

        set_pen = rospy.ServiceProxy(
            "/turtle1/set_pen",
            SetPen
        )

        # Turn pen off while positioning
        set_pen(
            off=1,
            r=255,
            g=255,
            b=255,
            width=2
        )

        # ----------------------------------------------------
        # Draw every wall cell as a square
        # ----------------------------------------------------

        for r in range(SIZE):

            for c in range(SIZE):

                if maze[r][c] != 1:
                    continue

                x, y = self.cell_position(
                    (r, c)
                )

                half = CELL_SIZE / 2.0

                # Start at bottom-left corner
                x1 = x - half
                y1 = y - half

                # Position turtle
                teleport(
                    x1,
                    y1,
                    0.0
                )

                # Turn pen ON
                set_pen(
                    off=0,
                    r=255,
                    g=255,
                    b=255,
                    width=2
                )

                # Draw square
                teleport(
                    x1 + CELL_SIZE,
                    y1,
                    0.0
                )

                teleport(
                    x1 + CELL_SIZE,
                    y1 + CELL_SIZE,
                    0.0
                )

                teleport(
                    x1,
                    y1 + CELL_SIZE,
                    0.0
                )

                teleport(
                    x1,
                    y1,
                    0.0
                )

                # Turn pen OFF before next positioning
                set_pen(
                    off=1,
                    r=255,
                    g=255,
                    b=255,
                    width=2
                )

        rospy.loginfo(
            "Maze drawing complete."
        )


    # ========================================================
    # MOVE TURTLE TO CELL
    # ========================================================

    def move_to_cell(self, cell):

        if self.pose is None:

            return

        target_x, target_y = self.cell_position(
            cell
        )

        rate = rospy.Rate(20)

        while not rospy.is_shutdown():

            dx = target_x - self.pose.x
            dy = target_y - self.pose.y

            distance = math.sqrt(
                dx * dx + dy * dy
            )

            if distance < 0.05:

                break

            target_angle = math.atan2(
                dy,
                dx
            )

            angle_error = (
                target_angle - self.pose.theta
            )

            while angle_error > math.pi:

                angle_error -= 2 * math.pi

            while angle_error < -math.pi:

                angle_error += 2 * math.pi

            cmd = Twist()

            cmd.linear.x = min(
                1.5,
                distance * 2.0
            )

            cmd.angular.z = angle_error * 4.0

            self.cmd_pub.publish(
                cmd
            )

            rate.sleep()

        self.cmd_pub.publish(
            Twist()
        )


    # ========================================================
    # FOLLOW SHORTEST PATH
    # ========================================================

    def follow_path(self, path):

        rospy.loginfo(
            "Following shortest path: %d cells",
            len(path)
        )

        for cell in path:

            if rospy.is_shutdown():

                return

            self.move_to_cell(
                cell
            )

        self.cmd_pub.publish(
            Twist()
        )

        rospy.loginfo(
            "Goal reached!"
        )


    # ========================================================
    # MAIN RUN FUNCTION
    # ========================================================

    def run(self):

        # Reset Turtlesim
        self.reset_turtlesim()

        rospy.sleep(1)

        # Generate maze
        maze = self.create_maze()

        # Solve maze using BFS
        path = self.solve_maze(
            maze
        )

        if path is None:

            rospy.logerr(
                "No path from START to GOAL."
            )

            return

        rospy.loginfo(
            "Shortest path found: %d cells",
            len(path)
        )

        # Draw maze
        self.draw_maze(
            maze
        )

        rospy.sleep(1)

        # ----------------------------------------------------
        # Move turtle to START
        # ----------------------------------------------------

        start_x, start_y = self.cell_position(
            START
        )

        rospy.wait_for_service(
            "/turtle1/teleport_absolute"
        )

        teleport = rospy.ServiceProxy(
            "/turtle1/teleport_absolute",
            TeleportAbsolute
        )

        # Pen off while positioning
        rospy.wait_for_service(
            "/turtle1/set_pen"
        )

        set_pen = rospy.ServiceProxy(
            "/turtle1/set_pen",
            SetPen
        )

        set_pen(
            off=1,
            r=255,
            g=255,
            b=255,
            width=3
        )

        teleport(
            start_x,
            start_y,
            0.0
        )

        # Turn pen on for solution path
        set_pen(
            off=0,
            r=255,
            g=255,
            b=255,
            width=3
        )

        rospy.sleep(1)

        rospy.loginfo(
            "Starting autonomous navigation..."
        )

        # Follow shortest path
        self.follow_path(
            path
        )

        rospy.loginfo(
            "Autonomous maze solving completed."
        )

        rospy.spin()


# ============================================================
# PROGRAM START
# ============================================================

if __name__ == "__main__":

    solver = MazeSolver()

    solver.run()
