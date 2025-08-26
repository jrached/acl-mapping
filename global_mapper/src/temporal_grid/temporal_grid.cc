// Temporal grid class for dynamic obstacle segmentation 
#include "temporal_grid/temporal_grid.h" 

// Fields: {is_free, occupied_duration, unoccupied_duration, last_occupied_time, last_unoccupied_time}

namespace temporal_grid 
{
TemporalGrid::TemporalGrid(const double origin[3], const double world_dimensions[3], const float resolution, float occupied_threshold, float unoccupied_threshold) 
: voxel_grid::VoxelGrid<std::vector<double>>(origin, world_dimensions, resolution), occupied_threshold_(occupied_threshold), unoccupied_threshold_(unoccupied_threshold), resolution_(resolution)
{
    offsets_ = this->generateOffsets(resolution_);
}

void TemporalGrid::UpdateTemporalInfo(const int ind, const bool is_occupied, const bool is_unknown, const double timestamp)
{
    timestamp_ = timestamp;
    std::vector temporal_info = this->ReadValue(ind); // {is_free, occupied_ruation, unoccupied_ruation, last_occupied_time, last_unoccupied_time}
    if (is_unknown == false)
    {
        // Initialize temporal grid voxels 
        if (temporal_info.size() == 0) 
        {
            temporal_info = {0.0, 0.0, 0.0, timestamp, timestamp, timestamp};
            this->WriteValue(ind, temporal_info); 
        }
        double is_free = temporal_info[0],
                occupied_duration = temporal_info[1], 
                unoccupied_duration = temporal_info[2],
                last_occupied_time = temporal_info[3], 
                last_unoccupied_time = temporal_info[4], 
                start_time = temporal_info[5];

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

        temporal_info = {is_free, occupied_duration, unoccupied_duration, last_occupied_time, last_unoccupied_time, start_time};
        this->WriteValue(ind, temporal_info);

        if (this->IsDynamic(ind, is_occupied))
        {
            double xyz[3];
            this->IndexToWorld(ind, xyz);
            if (xyz[2] > 0.5)
            {
                // if (occupied_duration > 0.0)
                if (true)
                {
                    std::cout << "Dynamic voxel (" << xyz[0] << ", " << xyz[1] << ", " << xyz[2] << ") with alive duration: " << timestamp - start_time << " and occupied duration: " << occupied_duration << std::endl;
                }
            }
        }
    }
    else 
    {
            temporal_info = {0.0, 0.0, 0.0, timestamp, timestamp, timestamp};
            this->WriteValue(ind, temporal_info); 
            return;
        }
}

void TemporalGrid::UpdateTemporalInfo(const int ixyz[3], const bool is_occupied, const bool is_unknown, const double timestamp)
{
    int ind = GridToIndex(ixyz);
    this->UpdateTemporalInfo(ind, is_occupied, is_unknown, timestamp);
}

void TemporalGrid::UpdateTemporalInfo(const double xyz[3], const bool is_occupied, const bool is_unknown, const double timestamp) 
{ 
    int ixyz[3];
    this->WorldToGrid(xyz, ixyz);
    if (this->IsInMap(ixyz))
    {
        this->UpdateTemporalInfo(ixyz, is_occupied, is_unknown, timestamp); 
    }
    else 
    {
        return; 
    }
}

bool TemporalGrid::AreNeighborsStatic(const int ixyz[3], int neigh_thresh)
{
    std::vector<double> temporal_info = this->GetTemporalInfo(ixyz);
    int not_free_neigh_counter = 0;
    int neigh_xyz[3]; 
    for (const auto offset : offsets_) 
    {
        neigh_xyz[0] = ixyz[0] + offset[0];
        neigh_xyz[1] = ixyz[1] + offset[1];
        neigh_xyz[2] = ixyz[2] + offset[2];
        
        // Check whether neighbors are not free space
        temporal_info = this->GetTemporalInfo(neigh_xyz);
        if (temporal_info.size() != 0 && temporal_info[0] == 0.0) 
        {
            not_free_neigh_counter++;
        }

        // If enough neighbors are not free space, this voxel is probably static
        if (not_free_neigh_counter >= neigh_thresh) 
        {
            return true;
        }
    }
    return false; // If voxel is occupied free space and not enough neighbors are static, then it is probably dynamic
}

bool TemporalGrid::IsDynamic(const int ind, bool is_occupied) 
{
    std::vector<double> voxel = this->ReadValue(ind);
    if (voxel.size() != 0 && voxel[0] == 1.0 && is_occupied) // If free and occupied (i.e. either dynamic or noise)
    {
        int ixyz[3];
        this->IndexToGrid(ind, ixyz); // TODO: Convert to grid indices instead 
        return !this->AreNeighborsStatic(ixyz, 1); // Filter out noise 
        return true;
    }
    return false; 
}

bool TemporalGrid::IsDynamic(const int ixyz[3], bool is_occupied) 
{   
    int ind = this->GridToIndex(ixyz); 
    return this->IsDynamic(ind, is_occupied); 
}

bool TemporalGrid::IsDynamic(const double xyz[3], bool is_occupied) 
{
    int ixyz[3];
    this->WorldToGrid(xyz, ixyz); 
    if (this->IsInMap(ixyz))
    {
        return this->IsDynamic(ixyz, is_occupied); 
    }
    return false; 

}

std::vector<double> TemporalGrid::GetTemporalInfo(const int ixyz[3]) const 
{
    int ind = this->GridToIndex(ixyz);
    return this->GetTemporalInfo(ind);
}

std::vector<double> TemporalGrid::GetTemporalInfo(const double xyz[3]) const 
{
    int ixyz[3]; 
    this->WorldToGrid(xyz, ixyz);
    if (this->IsInMap(ixyz)) 
    {
        return this->GetTemporalInfo(ixyz);
    }
    return std::vector<double>{}; // return empty vector if voxel is not in map
}

std::vector<double> TemporalGrid::GetTemporalInfo(const int ind) const 
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
    temporal_info = {is_free, 0.0, 0.0, timestamp_, timestamp_, timestamp_};
    this->WriteValue(ind, temporal_info);
}

void TemporalGrid::PostShiftOrigin(const std::vector<int>& slice_indexes)
{
    // Reinitialize incoming voxels
    for (const int index : slice_indexes)
    {
        double timestamp = timestamp_;
        std::vector<double> temporal_info;
        temporal_info = {0.0, 0.0, 0.0, timestamp, timestamp, timestamp};
        this->WriteValue(index, temporal_info); 
    }
}

void TemporalGrid::PreShiftOrigin(const std::vector<int>& slice_indexes)
{
    // Do nothing
}
} // namespace