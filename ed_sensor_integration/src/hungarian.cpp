#include "ed_sensor_integration/hungarian.h"
#include <algorithm>
#include <queue>
#include <cmath>

namespace ed_sensor_integration
{

double HungarianAlgorithm::solve(const std::vector<std::vector<double>>& cost_matrix,
                                 std::vector<int>& assignment)
{
    if (cost_matrix.empty())
    {
        assignment.clear();
        return 0.0;
    }

    int n = cost_matrix.size();        // Number of rows (measurements/clusters)
    int m = cost_matrix[0].size();     // Number of columns (entities)

    // Create a working copy of the cost matrix
    std::vector<std::vector<double>> cost = cost_matrix;

    // Make it square by padding with large costs if needed
    int size = std::max(n, m);
    if (n < size)
    {
        cost.resize(size);
        for (int i = n; i < size; ++i)
        {
            cost[i].resize(m, INF);
        }
    }
    if (m < size)
    {
        for (int i = 0; i < size; ++i)
        {
            cost[i].resize(size, INF);
        }
    }

    // Solve the square problem
    assignment.resize(n);
    double total_cost = solveSquare(cost, assignment, n, m);

    return total_cost;
}

double HungarianAlgorithm::solveSquare(std::vector<std::vector<double>>& cost,
                                       std::vector<int>& assignment,
                                       int n, int m)
{
    int size = cost.size();

    // Step 1: Subtract row minima
    subtractRowMinima(cost, size, size);

    // Step 2: Subtract column minima
    subtractColMinima(cost, size, size);

    // Initialize matching
    std::vector<int> row_mate(size, -1);  // row_mate[i] = j means row i is matched to column j
    std::vector<int> col_mate(size, -1);  // col_mate[j] = i means column j is matched to row i

    // Greedy initialization: find obvious zero-cost matches
    for (int i = 0; i < size; ++i)
    {
        for (int j = 0; j < size; ++j)
        {
            if (cost[i][j] == 0.0 && row_mate[i] == -1 && col_mate[j] == -1)
            {
                row_mate[i] = j;
                col_mate[j] = i;
                break;
            }
        }
    }

    // Main Hungarian algorithm loop
    std::vector<double> row_slack(size);
    std::vector<double> col_slack(size);
    std::vector<int> parent_row(size);
    std::vector<bool> row_visited(size);
    std::vector<bool> col_visited(size);

    for (int i = 0; i < size; ++i)
    {
        if (row_mate[i] == -1)
        {
            // Find augmenting path starting from unmatched row i
            bool found = findAugmentingPath(cost, row_mate, col_mate, row_slack, col_slack,
                                           parent_row, row_visited, col_visited, size, size, i);

            if (!found)
            {
                // This shouldn't happen with proper initialization
                // but if it does, the row remains unmatched
            }
        }
    }

    // Extract assignment and calculate total cost
    double total_cost = 0.0;
    for (int i = 0; i < n; ++i)
    {
        int j = row_mate[i];
        if (j >= m || cost[i][j] >= INF / 2.0)
        {
            assignment[i] = -1;  // No assignment (unmatched or invalid)
        }
        else
        {
            assignment[i] = j;
            total_cost += cost[i][j];
        }
    }

    return total_cost;
}

void HungarianAlgorithm::subtractRowMinima(std::vector<std::vector<double>>& cost, int n, int m)
{
    for (int i = 0; i < n; ++i)
    {
        double min_val = INF;
        for (int j = 0; j < m; ++j)
        {
            if (cost[i][j] < min_val)
                min_val = cost[i][j];
        }

        if (min_val > 0.0 && min_val < INF)
        {
            for (int j = 0; j < m; ++j)
            {
                if (cost[i][j] < INF)
                    cost[i][j] -= min_val;
            }
        }
    }
}

void HungarianAlgorithm::subtractColMinima(std::vector<std::vector<double>>& cost, int n, int m)
{
    for (int j = 0; j < m; ++j)
    {
        double min_val = INF;
        for (int i = 0; i < n; ++i)
        {
            if (cost[i][j] < min_val)
                min_val = cost[i][j];
        }

        if (min_val > 0.0 && min_val < INF)
        {
            for (int i = 0; i < n; ++i)
            {
                if (cost[i][j] < INF)
                    cost[i][j] -= min_val;
            }
        }
    }
}

bool HungarianAlgorithm::findAugmentingPath(const std::vector<std::vector<double>>& cost,
                                            std::vector<int>& row_mate,
                                            std::vector<int>& col_mate,
                                            std::vector<double>& row_slack,
                                            std::vector<double>& col_slack,
                                            std::vector<int>& parent_row,
                                            std::vector<bool>& row_visited,
                                            std::vector<bool>& col_visited,
                                            int n, int m, int unmatched_row)
{
    std::fill(row_visited.begin(), row_visited.end(), false);
    std::fill(col_visited.begin(), col_visited.end(), false);
    std::fill(parent_row.begin(), parent_row.end(), -1);
    std::fill(col_slack.begin(), col_slack.end(), INF);

    std::queue<int> q;
    q.push(unmatched_row);
    row_visited[unmatched_row] = true;

    while (!q.empty())
    {
        int i = q.front();
        q.pop();

        // Try to find a zero in this row
        for (int j = 0; j < m; ++j)
        {
            if (col_visited[j])
                continue;

            double slack = cost[i][j];

            if (slack < col_slack[j])
            {
                col_slack[j] = slack;
                parent_row[j] = i;

                if (slack == 0.0)
                {
                    col_visited[j] = true;

                    if (col_mate[j] == -1)
                    {
                        // Found an augmenting path! Update the matching.
                        while (j != -1)
                        {
                            int i_temp = parent_row[j];
                            int j_temp = row_mate[i_temp];
                            row_mate[i_temp] = j;
                            col_mate[j] = i_temp;
                            j = j_temp;
                        }
                        return true;
                    }
                    else
                    {
                        // Continue search from the matched row
                        int next_row = col_mate[j];
                        if (!row_visited[next_row])
                        {
                            row_visited[next_row] = true;
                            q.push(next_row);
                        }
                    }
                }
            }
        }
    }

    // No augmenting path found with current zero structure
    // This means the matching is optimal for the current dual variables
    return false;
}

} // namespace ed_sensor_integration
