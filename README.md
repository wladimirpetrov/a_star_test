# A* algorithm with the recommended pre-commit hooks

Here I worked with A* search algorithm for pathfinding in a grid with obstacles.

## Usage of black, isort, flake8

- **a_star_trial.py**: Before uploading to github I checked them by running through black, isort, flake8. I fixed the issues and then uploaded.

### Problems

The `.pre-commit-config.yaml` initially included hooks that required Python 3.9 or higher. Since I am using Python 3.8, I encountered compatibility issues and had to modify the configuration file to work with my environment.

### Dependencies

- numpy
- matplotlib

For time profiling, use https://pypi.org/project/tuna/.

#### Removed:

1. **`setup-cfg-fmt` hook**:
2. **`pyupgrade` hook**:
3. **`--py39-plus` arguments**:
