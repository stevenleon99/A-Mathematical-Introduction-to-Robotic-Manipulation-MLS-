import random

def initialize_grid(size):
    return [['⬜' for _ in range(size)] for _ in range(size)]

def random_walk(grid):
    size = len(grid)
    x, y = 0, 0
    end_x, end_y = size - 1, size - 1
    # Randomly walk from start to end
    while x != end_x or y != end_y:
        grid[x][y] = '⬜'
        if random.choice([True, False]) and x < end_x:
            x += 1
        elif y < end_y:
            y += 1

def add_obstacles(grid, num_obstacles):
    size = len(grid)
    count = 0
    while count < num_obstacles:
        r, c = random.randint(1, size-1), random.randint(1, size-1)
        if grid[r][c] == '⬜':
            grid[r][c] = '⬛'
            count += 1

def print_grid(grid):
    for row in grid:
        print(''.join(row))

def generate_grid(size=20, num_obstacles=50):
    grid = initialize_grid(size)
    random_walk(grid)  # Use a random walk to ensure a path
    add_obstacles(grid, num_obstacles)
    # Place the robot and the heart
    grid[size-1][0] = '🤖'
    grid[0][size-1] = '❤️'
    return grid

# Generate random grids for different sizes
sizes = [10, 20, 40, 80, 160, 320, 640]
for size in sizes:
    grid = generate_grid(size, size*2)
    print_grid(grid)
    if size < sizes[-1]:
        print()  # Print a newline between different grids for clarity
