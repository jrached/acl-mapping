// Temporal grid class for dynamic obstacle segmentation 
#pragma once 

#include "temporal_grid/temporal_grid.h" 

// Fields: {is_free, occupied_duration, unoccupied_duration, last_occupied_time, last_unoccupied_time}

namespace temporal_grid {
    TemporalGrid::TemporalGrid(const double origin[3], const double world_dimensions[3], const float resolution, float occupied_threshold, float unoccupied_threshold) 
    : voxel_grid::VoxelGrid<std::vector<double>>(origin, world_dimensions, resolution), occupied_threshold_(occupied_threshold), unoccupied_threshold_(unoccupied_threshold), resolution_(resolution)
    {
        // this->UpdateTemporalInfo(origin, true, 0.0);
    }

    void TemporalGrid::UpdateTemporalInfo(const int ind, const bool is_occupied, const double timestamp)
    {
        std::vector temporal_info = this->ReadValue(ind); // {is_free, occupied_ruation, unoccupied_ruation, last_occupied_time, last_unoccupied_time}
        if (temporal_info.size() == 0) 
        {
            temporal_info = {0.0, 0.0, 0.0, timestamp, timestamp};
            this->WriteValue(ind, temporal_info); 
        }
        double is_free = temporal_info[0],
               occupied_duration = temporal_info[1], 
               unoccupied_duration = temporal_info[2],
               last_occupied_time = temporal_info[3], 
               last_unoccupied_time = temporal_info[4];

        // Record occupancy duration 
        if (is_occupied) 
        {
            occupied_duration = timestamp - last_unoccupied_time;
            unoccupied_duration = 0.0; 
            last_occupied_time = timestamp; 
        } else
        {
            unoccupied_duration = timestamp - last_occupied_time; 
            occupied_duration = 0.0;
            last_unoccupied_time = timestamp;    
        }

        // Segment free and not free space 
        if (unoccupied_duration > unoccupied_threshold_) 
        {
            is_free = 1.0; 
        }
        if (occupied_duration > occupied_threshold_) 
        {
            is_free = 0.0;
        }
        
        temporal_info = {is_free, occupied_duration, unoccupied_duration, last_occupied_time, last_unoccupied_time};
        this->WriteValue(ind, temporal_info);
    }

    void TemporalGrid::UpdateTemporalInfo(const int ixyz[3], const bool is_occupied, const double timestamp)
    {
        int ind = GridToIndex(ixyz);
        this->UpdateTemporalInfo(ind, is_occupied, timestamp);
    }

    void TemporalGrid::UpdateTemporalInfo(const double xyz[3], const bool is_occupied, const double timestamp) 
    { 
        int ixyz[3];
        this->WorldToGrid(xyz, ixyz);
        if (this->IsInMap(ixyz))
        {
            this->UpdateTemporalInfo(ixyz, is_occupied, timestamp); 
        }
        else 
        {
            return; 
        }
    }

    // TODO: implement with KD-Tree
    void TemporalGrid::NearestNeighbors(const double xyz[3], bool is_occupied, int neigh_thresh)
    {
        std::vector<std::vector<double>> offsets = this->generateOffsets(resolution_);

        // If free and occupied (i.e. either dynamic or noise)
        std::vector<double> temporal_info = this->GetTemporalInfo(xyz);
        int not_free_neigh_counter = 0;
        if (is_occupied && temporal_info[0] == 1.0) 
        {
            for (const auto offset : offsets) 
            {
                int neigh_xyz[3]; 
                neigh_xyz[0] = xyz[0] + offset[0];
                neigh_xyz[1] = xyz[1] + offset[1];
                neigh_xyz[2] = xyz[2] + offset[2];
                
                // Check whether neighbors are not free space
                temporal_info = this->GetTemporalInfo(neigh_xyz);
                if (temporal_info[0] == 0.0) 
                {
                    not_free_neigh_counter++;
                }

                // If enough neighbors are not free space, this voxel must be not free space
                if (not_free_neigh_counter >= neigh_thresh) 
                {
                    this->SetFree(neigh_xyz, 0.0);
                    return;
                }
            }
        }
        else
        {
            return;
        }
    }

    bool TemporalGrid::IsDynamic(const int ind, bool is_occupied) 
    {
        double is_free = this->ReadValue(ind)[0]; 
        return is_free == 1.0 && is_occupied; 
    }

    bool TemporalGrid::IsDynamic(const int ixyz[3], bool is_occupied) 
    {
        int ind = this->GridToIndex(ixyz); 
        this->IsDynamic(ind, is_occupied); 
    }

    bool TemporalGrid::IsDynamic(const double xyz[3], bool is_occupied) 
    {
        int ixyz[3];
        this->WorldToGrid(xyz, ixyz); 
        if (this->IsInMap(ixyz))
        {
            return this->IsDynamic(ixyz, is_occupied); 
        }
        else 
        {
            return false; 
        }
    }

    std::vector<double> TemporalGrid::GetTemporalInfo(const int ixyz[3])
    {
        int ind = this->GridToIndex(ixyz);
        this->GetTemporalInfo(ind);
    }

    std::vector<double> TemporalGrid::GetTemporalInfo(const double xyz[3])
    {
        int ixyz[3]; 
        this->WorldToGrid(xyz, ixyz);
        if (this->IsInMap(ixyz)) 
        {
            return this->GetTemporalInfo(ixyz);
        }
        else 
        {
            return std::vector(0, 0.0); // return empty vector if voxel is not in map
        }
    }

    std::vector<double> TemporalGrid::GetTemporalInfo(const int ind)
    {
        return this->ReadValue(ind); 
    }

    void TemporalGrid::SetFree(const double xyz[3], float is_free)
    {
        int ixyz[3];
        this->WorldToGrid(xyz, ixyz);
        if (this->IsInMap(ixyz))
        {
            this->SetFree(ixyz, is_free);
        }
        else 
        {
            return; 
        }
    }

    void TemporalGrid::SetFree(const int ixyz[3], float is_free)
    {
        int ind = this->GridToIndex(ixyz);
        this->SetFree(ind, is_free);
    }

    void TemporalGrid::SetFree(const int ind, float is_free) 
    {
        std::vector<double> temporal_info = this->ReadValue(ind);
        temporal_info[0] = is_free;
        this->WriteValue(ind, temporal_info);
    }
}