import pygame
import math
import random
import sys
from queue import PriorityQueue as pq
from queue import LifoQueue as stk

pygame.init()

WIDTH = 800
STATUS_HEIGHT = 56
WIN = pygame.display.set_mode((WIDTH,WIDTH + STATUS_HEIGHT))
pygame.display.set_caption("A* / Dijkstra / Prim's Path Finding")
FONT = pygame.font.Font(None,28)
SMALL_FONT = pygame.font.Font(None,22)


RED = (255,0,0)
GREEN = (0,255,0)
BLUE = (0,0,255)
YELLOW = (255,255,0)
WHITE = (255,255,255)
BLACK = (0,0,0)
PURPLE = (128,0,128)
ORANGE = (255,165,0)
GREY = (128,128,128)
LIGHT_GREY = (230,230,230)
DARK_GREY = (80,80,80)
TURQUOISE = (64,224,208)

# ms to wait after each search step; Up/Down move through these
DELAYS = [0, 2, 5, 10, 20, 50, 100, 250, 500]
settings = {"algorithm": pygame.K_1, "delay": 3, "paused": False, "step": False, "found": None}

class Tile:
    def __init__(self,row,col,width,total_rows):
        self.row = row
        self.col = col
        self.x = row * width
        self.y = col * width
        self.color = WHITE
        self.neighbors = []
        self.width = width
        self.total_rows = total_rows

    def get_pos(self):
        return self.row, self.col

    def is_closed(self):
        return self.color == RED

    def is_open(self):
        return self.color == GREEN

    def is_barrier(self):
        return self.color == BLACK

    def is_start(self):
        return self.color == ORANGE

    def is_end(self):
        return self.color == TURQUOISE

    def is_path(self):
        return self.color == PURPLE

    def is_current(self):
        return self.color == YELLOW

    def reset(self):
        self.color = WHITE

    def make_closed(self):
        self.color = RED

    def make_open(self):
        self.color = GREEN

    def make_barrier(self):
        self.color = BLACK

    def make_start(self):
        self.color = ORANGE

    def make_end(self):
        self.color = TURQUOISE

    def make_path(self):
        self.color = PURPLE

    def make_current(self):
        self.color = YELLOW

    def draw(self,win):
        pygame.draw.rect(win,self.color,(self.x,self.y,self.width,self.width))
    
    def update_neighbors(self,grid):
        self.neighbors = []
        if self.row < self.total_rows - 1 and not grid[self.row +1][self.col].is_barrier():
            self.neighbors.append(grid[self.row + 1][self.col])
            
        if self.row > 0 and not grid[self.row -1][self.col].is_barrier():
            self.neighbors.append(grid[self.row - 1][self.col])

        if self.col < self.total_rows - 1 and not grid[self.row][self.col+1].is_barrier():
            self.neighbors.append(grid[self.row][self.col+1])

        if self.col > 0 and not grid[self.row][self.col-1].is_barrier():
            self.neighbors.append(grid[self.row][self.col - 1])

    def __lt__(self,other):
        return False

def h(p1,p2):
    x1, y1 = p1
    x2, y2 = p2

    return abs(x1 - x2 ) + abs(y1 - y2)

def reconstruct_path(parents,current,draw):
    stack = stk()
    while current in parents:
        current = parents[current]
        stack.put(current)
    stack.get()
    while not stack.empty():
        stack.get().make_path()
        draw()

def handle_setting_key(key):
    if key == pygame.K_UP:
        settings["delay"] = max(settings["delay"] - 1, 0)
    if key == pygame.K_DOWN:
        settings["delay"] = min(settings["delay"] + 1, len(DELAYS) - 1)
    if key == pygame.K_p:
        settings["paused"] = not settings["paused"]
    if key == pygame.K_RIGHT:
        settings["step"] = True

# Keys that stop a running search or maze and then take effect
INTERRUPT_KEYS = (pygame.K_c, pygame.K_m, pygame.K_1, pygame.K_2, pygame.K_3)

class Interrupted(Exception):
    def __init__(self, key):
        self.key = key

# Speed, pause and step keys keep working while a search is running
def handle_events():
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()
        if event.type == pygame.KEYDOWN:
            if event.key in INTERRUPT_KEYS:
                raise Interrupted(event.key)
            handle_setting_key(event.key)

# Puts the interrupting key back on the queue so the main loop handles it
def replay_key(interrupt):
    settings["paused"] = False
    pygame.event.post(pygame.event.Event(pygame.KEYDOWN, key=interrupt.key))

# Draws one search step, then waits for the delay, or while paused until Right is pressed
def show_step(draw):
    draw()
    resume_at = pygame.time.get_ticks() + DELAYS[settings["delay"]]
    while settings["paused"] or pygame.time.get_ticks() < resume_at:
        handle_events()
        if settings["paused"]:
            if settings["step"]:
                break
            draw()
            pygame.time.wait(30)
        else:
            pygame.time.wait(1)
    settings["step"] = False

def algorithm(draw, grid, start, end, heuristic=h):
    count = 0
    open_set = pq()
    open_set.put((0, count, start))
    parents = {}

    g_score = {tile:float("inf") for row in grid for tile in row}
    g_score[start] = 0

    f_score = {tile:float("inf") for row in grid for tile in row}
    f_score[start] = heuristic(start.get_pos(),end.get_pos())

    open_set_hash = {start}

    while not open_set.empty():
        handle_events()

        current = open_set.get()[2]
        open_set_hash.remove(current)

        if current == end:
            reconstruct_path(parents,end,draw)
            end.make_end()
            return True

        if current != start:
            current.make_current()

        for neighbor in current.neighbors:
            temp_g_score = g_score[current] + 1
            
            if temp_g_score < g_score[neighbor]:
                parents[neighbor] = current
                g_score[neighbor] = temp_g_score
                f_score[neighbor] = temp_g_score + heuristic(neighbor.get_pos(),end.get_pos())

                if neighbor not in open_set_hash:
                    count+=1
                    open_set.put((f_score[neighbor],count,neighbor))
                    open_set_hash.add(neighbor)
                    neighbor.make_open()
        draw()

        if current != start:
            current.make_closed()
    return False

# Dijkstra is A* without a heuristic
def dijkstra(draw, grid, start, end):
    return algorithm(draw, grid, start, end, lambda p1, p2: 0)

# Grows a minimum spanning tree from start over random edge weights.
# Stops once end joins the tree, so the path shown is the tree's route,
# which is usually longer than the shortest path.
def prims(draw, grid, start, end):
    count = 0
    open_set = pq()
    open_set.put((0, count, start))
    parents = {}
    weights = {}

    key = {tile:float("inf") for row in grid for tile in row}
    key[start] = 0

    in_tree = set()

    while not open_set.empty():
        handle_events()

        current = open_set.get()[2]
        if current in in_tree:
            continue
        in_tree.add(current)

        if current == end:
            reconstruct_path(parents,end,draw)
            end.make_end()
            return True

        if current != start:
            current.make_current()

        for neighbor in current.neighbors:
            weight =weights.setdefault(frozenset((current, neighbor)), random.random())

            if neighbor not in in_tree and weight < key[neighbor]:
                parents[neighbor] = current
                key[neighbor] = weight
                count+=1
                open_set.put((weight,count,neighbor))
                neighbor.make_open()
        draw()

        if current != start:
            current.make_closed()
    return False

# Randomized Prim's maze: passages on even rows/cols, walls between them
def generate_maze(draw, grid):
    rows = len(grid)
    for row in grid:
        for tile in row:
            tile.make_barrier()

    def cells_around(row, col):
        for dr, dc in ((2,0),(-2,0),(0,2),(0,-2)):
            if 0 <= row + dr < rows and 0 <= col + dc < rows:
                yield row + dr, col + dc

    grid[0][0].reset()
    frontier = [((0,0),cell) for cell in cells_around(0,0)]

    while frontier:
        handle_events()

        (from_row, from_col), (row, col) = frontier.pop(random.randrange(len(frontier)))
        if grid[row][col].is_barrier():
            grid[(from_row + row)//2][(from_col + col)//2].reset()
            grid[row][col].reset()
            frontier.extend(((row,col),cell) for cell in cells_around(row,col) if grid[cell[0]][cell[1]].is_barrier())
            draw()

    last = (rows - 1)//2*2
    start, end = grid[0][0], grid[last][last]
    start.make_start()
    end.make_end()
    return start, end

def clear_search(grid, start, end):
    for row in grid:
        for tile in row:
            if tile.is_open() or tile.is_closed() or tile.is_current() or tile.is_path():
                tile.reset()
    if start:
        start.make_start()
    if end:
        end.make_end()
    settings["found"] = None

def make_grid(rows,width):
    grid = []
    gap = width // rows

    for i in range(rows):
        grid.append([])
        for j in range(rows):
            tile = Tile(i,j,gap,rows)
            grid[i].append(tile)

    return grid

def draw_grid(win,rows,width):
    gap = width // rows
    for i in range(rows):
        pygame.draw.line(win,GREY,(0,i*gap),(width,i*gap))
        pygame.draw.line(win,GREY,(i*gap,0),(i*gap,width))

def draw(win,grid,rows,width):
    win.fill(WHITE)
    
    for row in grid:
        for tile in row:
            tile.draw(win)

    draw_grid(win,rows,width)
    draw_status(win,grid,width)
    pygame.display.update()

def draw_status(win,grid,width):
    pygame.draw.rect(win,LIGHT_GREY,(0,width,width,STATUS_HEIGHT))

    tiles = [tile for row in grid for tile in row]
    explored = sum(tile.is_closed() or tile.is_current() or tile.is_path() for tile in tiles)
    status = f"{ALGORITHMS[settings['algorithm']][0]}    Explored: {explored}"
    if settings["found"] is True:
        status += f"    Path: {sum(tile.is_path() for tile in tiles) + 1} steps"
    elif settings["found"] is False:
        status += "    No path"
    win.blit(FONT.render(status,True,BLACK),(10,width + 8))

    if settings["paused"]:
        speed = FONT.render("PAUSED  (Right: step)",True,RED)
    else:
        speed = FONT.render(f"Delay: {DELAYS[settings['delay']]} ms/step",True,BLACK)
    win.blit(speed,(width - speed.get_width() - 10,width + 8))

    keys = "1/2/3: algorithm   Space: run   P: pause   Up/Down: speed   M: maze   C: clear"
    win.blit(SMALL_FONT.render(keys,True,DARK_GREY),(10,width + 34))


def get_clicked_pos(pos,rows,width):
    gap = width // rows
    y, x = pos
    
    row = y // gap
    col = x // gap
    return row, col

ALGORITHMS = {
    pygame.K_1: ("A*", algorithm),
    pygame.K_2: ("Dijkstra", dijkstra),
    pygame.K_3: ("Prim's", prims),
}

def main(win,width):
    ROWS = 50
    grid = make_grid(ROWS,width)
    redraw = lambda : draw(win,grid,ROWS,width)

    start = None
    end = None

    run = True
    while run:
        draw(win,grid,ROWS,width)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                 run = False

            pos = pygame.mouse.get_pos()
            on_grid = pos[1] < width

            if pygame.mouse.get_pressed()[0] and on_grid:

                row, col = get_clicked_pos(pos,ROWS,width)
                tile = grid[row][col]
                if not start and tile != end:
                    start = tile
                    start.make_start()
                elif not end and tile != start:
                    end = tile
                    end.make_end()
                elif tile!= end and tile!= start:
                    tile.make_barrier()

            elif pygame.mouse.get_pressed()[2] and on_grid:

                row, col = get_clicked_pos(pos,ROWS,width)
                tile = grid[row][col]
                tile.reset()
                if tile == start:
                    start = None
                elif tile == end:
                    end = None

            if event.type == pygame.KEYDOWN:
                handle_setting_key(event.key)

                if event.key == pygame.K_SPACE and start and end:
                    clear_search(grid,start,end)
                    settings["step"] = False
                    for row in grid:
                        for tile in row:
                            tile.update_neighbors(grid)

                    algorithm_to_run = ALGORITHMS[settings["algorithm"]][1]
                    try:
                        settings["found"] = algorithm_to_run(lambda : show_step(redraw),grid,start,end)
                        end.make_end()
                    except Interrupted as interrupt:
                        replay_key(interrupt)

                if event.key in ALGORITHMS:
                    settings["algorithm"] = event.key
                    clear_search(grid,start,end)

                if event.key == pygame.K_m:
                    grid = make_grid(ROWS,width)
                    start = None
                    end = None
                    settings["found"] = None
                    try:
                        start, end = generate_maze(redraw,grid)
                    except Interrupted as interrupt:
                        replay_key(interrupt)

                if event.key == pygame.K_c:
                    start = None
                    end = None
                    grid = make_grid(ROWS,width)
                    settings["found"] = None
    pygame.quit()
    

main(WIN,WIDTH)

    




        
