# Grid-Based Maze Generation and Pathfinding Performance Evaluation

This project implements the A* pathfinding algorithm to navigate through a series of grid-based mazes. The project consists of python scripts that generate mazes (grids), and MATLAB scripts that read maze configurations from a file, and apply the A* algorithm using various heuristics to find the shortest path from a defined start point to a goal point.

![Graph](./.data/Graph.png)

## Project Structure

- `main.m`: Main script that orchestrates the reading of grids, application of the A\* algorithm, and plotting of results.
- `read_grids_from_file.m`: Reads grid configurations from a specified text file, where each grid is defined by its obstacles and empty spaces.
- `create_puzzle.m`: Function to generate a list of grids with a guaranteed path and random obstacles.
- `a_star.m`: Implementation of the A\* algorithm with support for Manhattan, Euclidean, and Dijkstra's heuristic calculations.
- `sample.txt`: Test sample file created as follows -

```bash
python3 create_puzzle.py > sample.txt
```

## Running the Project

### Running the Scripts

1. **Clone or download the repository** to your local machine.
2. **Open MATLAB** and navigate to the project's directory.
3. **Run the `main.m` script**:
   - Open the script in MATLAB's editor.
   - Press the 'Run' button or type `main` in the Command Window.
4. **View the results**:
   - The script will generate plots comparing the performance of different heuristics.

### Customizing Grids

To test different grid configurations:

- Modify the `sample.txt` file to include new grid setups. Follow the existing format where `⬜` represents an open space and `⬛` represents an obstacle. `🤖` and `❤️` should be used to denote the start and goal positions, respectively.
- You may also use the above bash command to generate new sample files.
