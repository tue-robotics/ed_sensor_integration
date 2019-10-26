#include "ed_sensor_integration/association_matrix.h"
#include <algorithm>

#include <iostream>

namespace ed_sensor_integration
{

bool compareEntries(const AssociationMatrix::Entry& e1, const AssociationMatrix::Entry& e2)
{
    return e1.probability > e2.probability;
}

//int AssociationMatrix::NO_ASSIGNMENT = -1;

// ----------------------------------------------------------------------------------------------------

AssociationMatrix::AssociationMatrix(unsigned int num_measurements) : matrix_(num_measurements), i_max_entity_(0)
{
}

// ----------------------------------------------------------------------------------------------------

AssociationMatrix::~AssociationMatrix()
{
}

// ----------------------------------------------------------------------------------------------------

void AssociationMatrix::setEntry(int i_measurement, int i_entity, double prob)
{
    if (prob <= 0)
        return;

    std::vector<Entry>& msr_row = matrix_[i_measurement];
    msr_row.push_back(Entry(i_measurement, i_entity, prob));

    i_max_entity_ = std::max(i_max_entity_, i_entity);
}

// ----------------------------------------------------------------------------------------------------


bool AssociationMatrix::calculateBestAssignment(Assignment& assig)
{
    // Sort all rows (highest prob first)
    for(unsigned int i = 0; i < matrix_.size(); ++i)
    {
        std::vector<Entry>& msr_row = matrix_[i];
        std::sort(msr_row.begin(), msr_row.end(), compareEntries);

        // Add dummy entry
        msr_row.push_back(Entry(i, -1, 1e-9));
    }

    // Initialize
    std::vector<int> assig_indexes(matrix_.size(), 0);

    while(true)
    {
        // Check if the assignment is valid
        bool valid = true;
        std::vector<int> entity_used(i_max_entity_ + 1, 0);
        for(unsigned int i = 0; i < assig_indexes.size(); ++i)
        {
            const Entry& entry = matrix_[i][assig_indexes[i]];
            if (entry.i_entity >= 0)
            {
                ++entity_used[entry.i_entity];

                // Check if entity has been assigned double. If so, this assignment is not valid
                if (entity_used[entry.i_entity] > 1)
                    valid = false;
            }

            if (!valid)
                break;
        }

        // If we found a valid assignment, we can stop
        if (valid)
            break;

        double smallest_prob_diff = 1e9;
        int i_smallest_prob_diff = -1;
        for(unsigned int i = 0; i < assig_indexes.size(); ++i)
        {
            std::vector<Entry>& msr_row = matrix_[i];
            int j = assig_indexes[i];

            if (j + 1 < msr_row.size())
            {
                double prob_diff = msr_row[j].probability / msr_row[j + 1].probability;
                if (prob_diff < smallest_prob_diff)
                {
                    i_smallest_prob_diff = i;
                    smallest_prob_diff = prob_diff;
                }
            }
        }

        if (i_smallest_prob_diff < 0)
        {
            // Found no next step to take, so we're done and didn't find a valid assignment
            return false;
        }

        // Otherwise, step the assignment with the smallest probability diff
        ++assig_indexes[i_smallest_prob_diff];
    }

    assig.resize(matrix_.size());
    for(unsigned int i = 0; i < assig_indexes.size(); ++i)
    {
        assig[i] = matrix_[i][assig_indexes[i]].i_entity;
    }

    return true;
}

}

