# pathfinding_multi.py
# Dialog-based pathfinding visualizer with A*, BFS, DFS
# Based on the Tk-dialog repo you provided; BFS/DFS added + dropdown + keyboard shortcuts.

import pygame
import sys
import math
import os
from tkinter import *
from tkinter import ttk
from tkinter import messagebox
from collections import deque

# ---- PYGAME SETUP ----
pygame.init()
WIDTH = 800
screen = pygame.display.set_mode((WIDTH, WIDTH))
pygame.display.set_caption("A* / BFS / DFS Path Finding (dialog version)")

# Colors
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
GREY = (220, 220, 220)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
MAGENTA = (255, 8, 127)

# Grid configuration (keep same as original)
cols = 50
rows = 50
w = WIDTH / cols
h = WIDTH / rows

# ---- Spot class (same semantics as your repo) ----
class Spot:
    def __init__(self, x, y):
        self.i = x
        self.j = y
        self.f = 0.0
        self.g = 0.0
        self.h = 0.0
        self.neighbors = []
        self.previous = None
        self.obs = False
        self.closed = False
        self.value = 1

    def show(self, color, border):
        pygame.draw.rect(screen, color, (self.i * w, self.j * h, w, h), border)

    def addNeighbors(self, grid):
        # reset then add four-directional neighbors if not blocked
        self.neighbors = []
        i = self.i
        j = self.j
        # Right
        if i < cols - 1 and not grid[i + 1][j].obs:
            self.neighbors.append(grid[i + 1][j])
        # Left
        if i > 0 and not grid[i - 1][j].obs:
            self.neighbors.append(grid[i - 1][j])
        # Down
        if j < rows - 1 and not grid[i][j + 1].obs:
            self.neighbors.append(grid[i][j + 1])
        # Up
        if j > 0 and not grid[i][j - 1].obs:
            self.neighbors.append(grid[i][j - 1])

# ---- Build grid ----
grid = [[Spot(i, j) for j in range(rows)] for i in range(cols)]

# Default start/end (will be overwritten by dialog)
start = grid[12][5]
end = grid[3][6]

# Draw initial grid & optional borders (kept from your repo)
def draw_grid_background():
    screen.fill(BLACK)
    for i in range(cols):
        for j in range(rows):
            spot = grid[i][j]
            if spot.obs:
                spot.show(BLACK, 0)
            else:
                spot.show(WHITE, 1)
    # grid lines
    for x in range(cols):
        pygame.draw.line(screen, GREY, (x * w, 0), (x * w, WIDTH))
    for y in range(rows):
        pygame.draw.line(screen, GREY, (0, y * h), (WIDTH, y * h))
    pygame.display.flip()

# Mark border obstacles (original repo did this)
for i in range(rows):
    grid[0][i].obs = True
    grid[cols - 1][i].obs = True
    grid[i][0].obs = True
    grid[i][rows - 1].obs = True

draw_grid_background()

# ---- Tkinter dialog to get Start, End, ShowSteps, Algorithm ----
window = Tk()
window.title("Pathfinding Input")

Label(window, text='Start(x,y): ').grid(row=0, column=0, pady=3)
startBox = Entry(window)
startBox.grid(row=0, column=1, pady=3)

Label(window, text='End(x,y): ').grid(row=1, column=0, pady=3)
endBox = Entry(window)
endBox.grid(row=1, column=1, pady=3)

var_steps = IntVar(value=1)
showPath = ttk.Checkbutton(window, text='Show Steps :', onvalue=1, offvalue=0, variable=var_steps)
showPath.grid(columnspan=2, row=2)

Label(window, text='Algorithm:').grid(row=3, column=0, pady=3)
algo_var = StringVar(value="A*")
algo_dropdown = ttk.Combobox(window, textvariable=algo_var, values=["A*", "BFS", "DFS"], state="readonly", width=10)
algo_dropdown.grid(row=3, column=1, pady=3)

def onsubmit():
    global start, end, selected_algorithm
    st = startBox.get().strip()
    ed = endBox.get().strip()
    # allow blank or invalid entries -> keep defaults, but attempt to parse
    try:
        ssplit = [int(x.strip()) for x in st.split(',') if x.strip() != '']
        if len(ssplit) == 2:
            sx, sy = ssplit
            if 0 <= sx < cols and 0 <= sy < rows:
                start = grid[sx][sy]
    except Exception:
        pass
    try:
        esplit = [int(x.strip()) for x in ed.split(',') if x.strip() != '']
        if len(esplit) == 2:
            ex, ey = esplit
            if 0 <= ex < cols and 0 <= ey < rows:
                end = grid[ex][ey]
    except Exception:
        pass
    selected_algorithm = algo_var.get()
    window.quit()
    window.destroy()

submit = Button(window, text='Submit', command=onsubmit)
submit.grid(columnspan=2, row=4, pady=6)

# modal-like behavior: update until submit pressed
window.update()
window.mainloop()

# Re-draw after dialog
draw_grid_background()
start.show(MAGENTA, 0)
end.show(MAGENTA, 0)
pygame.display.flip()

# ---- Utility functions ----
def heurisitic(n, e):
    # Euclidean as in your repo
    return math.hypot(n.i - e.i, n.j - e.j)

def prepare_grid():
    for i in range(cols):
        for j in range(rows):
            grid[i][j].addNeighbors(grid)
            grid[i][j].g = float('inf')
            grid[i][j].h = 0.0
            grid[i][j].f = float('inf')
            grid[i][j].previous = None
            grid[i][j].closed = False

def reconstruct_path(end_node):
    # walk backwards via previous and color path (blue)
    cur = end_node
    length = 0
    while cur.previous:
        cur = cur.previous
        cur.closed = False
        cur.show(BLUE, 0)
        pygame.display.flip()
        length += 1
    return length

# ---- Algorithms ----

def run_astar(show_steps_flag):
    prepare_grid()
    openSet = [start]
    start.g = 0.0
    start.h = heurisitic(start, end)
    start.f = start.h

    closedSet = []

    running = True
    while running and len(openSet) > 0:
        # handle pygame events to keep UI responsive
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                pygame.quit()
                sys.exit()

        # find lowest f in openSet
        lowestIndex = 0
        for i in range(len(openSet)):
            if openSet[i].f < openSet[lowestIndex].f:
                lowestIndex = i
        current = openSet[lowestIndex]

        if current == end:
            distance = reconstruct_path(current)
            end.show(MAGENTA, 0)
            start.show(MAGENTA, 0)
            pygame.display.flip()
            return distance

        # move current from open to closed
        openSet.pop(lowestIndex)
        closedSet.append(current)

        for neighbor in current.neighbors:
            if neighbor in closedSet:
                continue

            tempG = current.g + neighbor.value
            if neighbor in openSet:
                if neighbor.g > tempG:
                    neighbor.g = tempG
                    neighbor.previous = current
            else:
                neighbor.g = tempG
                neighbor.previous = current
                openSet.append(neighbor)

            neighbor.h = heurisitic(neighbor, end)
            neighbor.f = neighbor.g + neighbor.h

        # visualization
        if show_steps_flag:
            for node in openSet:
                if node not in (start, end):
                    node.show(GREEN, 0)
            for node in closedSet:
                if node not in (start, end):
                    node.show(RED, 0)
            pygame.display.flip()

        current.closed = True

    return None  # no path

def run_bfs(show_steps_flag):
    prepare_grid()
    queue = deque()
    queue.append(start)
    start.g = 0
    visited = set([start])
    parent_map = {}
    found = False

    while queue:
        # event loop
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                pygame.quit()
                sys.exit()

        current = queue.popleft()
        if current == end:
            found = True
            break

        for neighbor in current.neighbors:
            if neighbor not in visited:
                visited.add(neighbor)
                parent_map[neighbor] = current
                neighbor.previous = current
                neighbor.g = current.g + 1
                queue.append(neighbor)

        if show_steps_flag:
            # show open (queue) and closed (visited)
            for node in queue:
                if node not in (start, end):
                    node.show(GREEN, 0)
            for node in visited:
                if node not in (start, end):
                    node.show(RED, 0)
            pygame.display.flip()

        current.closed = True

    if found:
        # reconstruct using previous pointers (set by parent_map)
        return reconstruct_path(end)
    return None

def run_dfs(show_steps_flag):
    prepare_grid()
    stack = [start]
    visited = set([start])
    found = False

    while stack:
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                pygame.quit()
                sys.exit()

        current = stack.pop()
        if current == end:
            found = True
            break

        for neighbor in current.neighbors:
            if neighbor not in visited:
                visited.add(neighbor)
                neighbor.previous = current
                neighbor.g = current.g + 1
                stack.append(neighbor)

        if show_steps_flag:
            # stack as open set, visited as closed
            for node in stack:
                if node not in (start, end):
                    node.show(GREEN, 0)
            for node in visited:
                if node not in (start, end):
                    node.show(RED, 0)
            pygame.display.flip()

        current.closed = True

    if found:
        return reconstruct_path(end)
    return None

# ---- Main loop: allow keyboard shortcuts and runtime drawing of obstacles ----
show_steps_flag = bool(var_steps.get())
selected_algorithm = locals().get('selected_algorithm', 'A*')  # from dialog if set

running = True
searching = False

def reset_visual_states():
    for i in range(cols):
        for j in range(rows):
            cell = grid[i][j]
            cell.g = 0
            cell.h = 0
            cell.f = 0
            cell.previous = None
            cell.closed = False

# draw initial marks
draw_grid_background()
start.show(MAGENTA, 0)
end.show(MAGENTA, 0)
pygame.display.flip()

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
            break

        # Left mouse: draw obstacle (unless it's start or end)
        if pygame.mouse.get_pressed()[0]:
            pos = pygame.mouse.get_pos()
            gx = int(pos[0] // w)
            gy = int(pos[1] // h)
            if 0 <= gx < cols and 0 <= gy < rows:
                cell = grid[gx][gy]
                if cell != start and cell != end:
                    cell.obs = True
                    cell.show(BLACK, 0)
                    pygame.display.flip()

        # Right mouse: clear cell (remove obstacle or unset start/end)
        if pygame.mouse.get_pressed()[2]:
            pos = pygame.mouse.get_pos()
            gx = int(pos[0] // w)
            gy = int(pos[1] // h)
            if 0 <= gx < cols and 0 <= gy < rows:
                cell = grid[gx][gy]
                cell.obs = False
                cell.previous = None
                cell.g = cell.h = cell.f = 0
                cell.closed = False
                if cell == start:
                    start = None
                if cell == end:
                    end = None
                draw_grid_background()
                if start:
                    start.show(MAGENTA, 0)
                if end:
                    end.show(MAGENTA, 0)
                pygame.display.flip()

        if event.type == pygame.KEYDOWN:
            # Keyboard overrides for algorithm selection
            if event.key == pygame.K_SPACE:
                # A*
                algo_to_run = "A*"
            elif event.key == pygame.K_b:
                algo_to_run = "BFS"
            elif event.key == pygame.K_d:
                algo_to_run = "DFS"
            elif event.key == pygame.K_c:
                # clear entire grid (preserve border obs)
                for i in range(cols):
                    for j in range(rows):
                        grid[i][j].obs = False
                        grid[i][j].previous = None
                        grid[i][j].g = grid[i][j].h = grid[i][j].f = 0
                        grid[i][j].closed = False
                # re-enable border obs as original repo
                for i in range(rows):
                    grid[0][i].obs = True
                    grid[cols - 1][i].obs = True
                    grid[i][0].obs = True
                    grid[i][rows - 1].obs = True
                draw_grid_background()
                start.show(MAGENTA, 0)
                end.show(MAGENTA, 0)
                pygame.display.flip()
                algo_to_run = None
            elif event.key == pygame.K_s:
                show_steps_flag = not show_steps_flag
                print("Show steps:", show_steps_flag)
                algo_to_run = None
            else:
                algo_to_run = None

            # If user pressed an algorithm key, run it
            if 'algo_to_run' in locals() and algo_to_run:
                # If user selected via dropdown earlier, keyboard overrides
                selected_algorithm = algo_to_run

                # ensure start and end are set
                if not start or not end:
                    print("Start or End not set.")
                    continue

                # prepare neighbor lists
                for i in range(cols):
                    for j in range(rows):
                        grid[i][j].addNeighbors(grid)

                # run the selected algorithm
                if selected_algorithm == "A*":
                    result = run_astar(show_steps_flag)
                elif selected_algorithm == "BFS":
                    result = run_bfs(show_steps_flag)
                elif selected_algorithm == "DFS":
                    result = run_dfs(show_steps_flag)
                else:
                    result = None

                # Show final UI and dialog
                if result is not None:
                    end.show(MAGENTA, 0)
                    start.show(MAGENTA, 0)
                    pygame.display.flip()
                    # message and rerun option
                    Tk().wm_withdraw()
                    result_box = messagebox.askokcancel('Program Finished', (
                        'The program finished, the shortest distance \n to the path is ' + str(round(result, 2)) + ' blocks away, \n would you like to re run the program?'))
                    if result_box:
                        os.execl(sys.executable, sys.executable, *sys.argv)
                    else:
                        # pause until user presses any key then exit normally
                        paused = True
                        while paused:
                            for evp in pygame.event.get():
                                if evp.type == pygame.KEYDOWN or evp.type == pygame.QUIT:
                                    paused = False
                                    break
                        pygame.quit()
                        sys.exit()
                else:
                    Tk().wm_withdraw()
                    messagebox.showinfo('No Path', 'No path could be found with the current obstacle layout.')
                    # re-draw baseline
                    draw_grid_background()
                    start.show(MAGENTA, 0)
                    end.show(MAGENTA, 0)
                    pygame.display.flip()

    # small delay to keep UI responsive
    pygame.time.delay(10)

pygame.quit()
sys.exit()
