import pygame
import random
import heapq

# Grid and bot settings
GRID_SIZE = 20
CELL_SIZE = 30
BOT_RADIUS = 10
WINDOW_SIZE = GRID_SIZE * CELL_SIZE
BOT_COUNT = 3

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
BLUE = (0, 0, 255)
GREEN = (0, 255, 0)
RED = (255, 0, 0)

# Directions
DIRS = [(0, 1), (1, 0), (0, -1), (-1, 0)]

def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def a_star(grid, start, goal):
    frontier = []
    heapq.heappush(frontier, (0, start))
    came_from = {start: None}
    cost_so_far = {start: 0}

    while frontier:
        _, current = heapq.heappop(frontier)
        if current == goal:
            break
        for dx, dy in DIRS:
            next_node = (current[0] + dx, current[1] + dy)
            if 0 <= next_node[0] < GRID_SIZE and 0 <= next_node[1] < GRID_SIZE:
                new_cost = cost_so_far[current] + 1
                if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                    cost_so_far[next_node] = new_cost
                    priority = new_cost + heuristic(goal, next_node)
                    heapq.heappush(frontier, (priority, next_node))
                    came_from[next_node] = current

    path = []
    node = goal
    while node != start:
        path.append(node)
        node = came_from.get(node)
        if node is None:
            return []  # No path found
    path.reverse()
    return path

class Bot:
    def __init__(self, start, goal, color):
        self.pos = start
        self.goal = goal
        self.path = []
        self.color = color

    def plan_path(self, grid):
        self.path = a_star(grid, self.pos, self.goal)

    def move_step(self):
        if self.path:
            self.pos = self.path.pop(0)

def draw_grid(screen):
    for x in range(0, WINDOW_SIZE, CELL_SIZE):
        pygame.draw.line(screen, BLACK, (x, 0), (x, WINDOW_SIZE))
    for y in range(0, WINDOW_SIZE, CELL_SIZE):
        pygame.draw.line(screen, BLACK, (0, y), (WINDOW_SIZE, y))

def draw_bot(screen, bot):
    x, y = bot.pos
    pygame.draw.circle(screen, bot.color, (x * CELL_SIZE + CELL_SIZE // 2, y * CELL_SIZE + CELL_SIZE // 2), BOT_RADIUS)

pygame.init()
screen = pygame.display.set_mode((WINDOW_SIZE, WINDOW_SIZE))
pygame.display.set_caption("Bot Pathfinding Simulation")
clock = pygame.time.Clock()

grid = [[0 for _ in range(GRID_SIZE)] for _ in range(GRID_SIZE)]
bots = []
colors = [BLUE, GREEN, RED]

for i in range(BOT_COUNT):
    while True:
        start = (random.randint(0, GRID_SIZE-1), random.randint(0, GRID_SIZE-1))
        goal = (random.randint(0, GRID_SIZE-1), random.randint(0, GRID_SIZE-1))
        if start != goal:
            break
    bot = Bot(start, goal, colors[i % len(colors)])
    bot.plan_path(grid)
    bots.append(bot)

running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    screen.fill(WHITE)
    draw_grid(screen)

    for bot in bots:
        draw_bot(screen, bot)
        bot.move_step()

    pygame.display.flip()
    clock.tick(3)

pygame.quit()

