#include "color_table.h"

#include <fstream>

int ColorTable::RESOLUTION = 32;
int ColorTable::NUM_COLORS = 11;
int ColorTable::STEP = 256 / RESOLUTION;


// ----------------------------------------------------------------------------------------------------

ColorTable::ColorTable()
{
}

// ----------------------------------------------------------------------------------------------------

ColorTable::~ColorTable()
{
}

// ----------------------------------------------------------------------------------------------------

bool ColorTable::readFromFile(const std::string& filename)
{
    std::fstream file(filename.c_str(), std::ios_base::in);

    if (!file.is_open())
        return false;

    table_.resize(RESOLUTION * RESOLUTION * RESOLUTION * NUM_COLORS);

    int k = 0;
    for(int b = 0; b < RESOLUTION; ++b)
    {
        for(int g = 0; g < RESOLUTION; ++g)
        {
            for(int r = 0; r < RESOLUTION; ++r)
            {
                // Skip first three values
                float v1, v2, v3;
                file >> v1 >> v2 >> v3;

                for(int i = 0; i < NUM_COLORS; ++i)
                    file >> table_[k + i];

                k += NUM_COLORS;
            }
        }
    }
}
