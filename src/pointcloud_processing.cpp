#include "pointcloud_processing.h"


void apply_color(PointCloudT::Ptr cloud)
{
    int filtering_axis_(2);
    int color_mode_(4);

    // Find the minimum and maximum values along the selected axis
    double min, max;
    // Set an initial value
    switch (filtering_axis_)
    {
    case 0: // x
        min = (*cloud)[0].x;
        max = (*cloud)[0].x;
        break;
    case 1: // y
        min = (*cloud)[0].y;
        max = (*cloud)[0].y;
        break;
    default: // z
        min = (*cloud)[0].z;
        max = (*cloud)[0].z;
        break;
    }

    // Search for the minimum/maximum
    for (PointCloudT::iterator cloud_it = cloud->begin(); cloud_it != cloud->end(); ++cloud_it)
    {
        switch (filtering_axis_)
        {
        case 0: // x
            if (min > cloud_it->x)
                min = cloud_it->x;

            if (max < cloud_it->x)
                max = cloud_it->x;
            break;
        case 1: // y
            if (min > cloud_it->y)
                min = cloud_it->y;

            if (max < cloud_it->y)
                max = cloud_it->y;
            break;
        default: // z
            if (min > cloud_it->z)
                min = cloud_it->z;

            if (max < cloud_it->z)
                max = cloud_it->z;
            break;
        }
    }

    // Compute LUT scaling to fit the full histogram spectrum
    double lut_scale = 255.0 / (max - min); // max is 255, min is 0

    if (min == max)      // In case the cloud is flat on the chosen direction (x,y or z)
        lut_scale = 1.0; // Avoid rounding error in boost

    for (PointCloudT::iterator cloud_it = cloud->begin(); cloud_it != cloud->end(); ++cloud_it)
    {

        int value;
        switch (filtering_axis_)
        {
        case 0: // x
            value = std::lround((cloud_it->x - min) * lut_scale);
            break;
        case 1: // y
            value = std::lround((cloud_it->y - min) * lut_scale);
            break;
        default: // z
            value = std::lround((cloud_it->z - min) * lut_scale);
            break;
        }

        // Apply color to the cloud
        switch (color_mode_)
        {
        case 0:
            // Blue (= min) -> Red (= max)
            cloud_it->r = value;
            cloud_it->g = 0;
            cloud_it->b = 255 - value;
            break;
        case 1:
            // Green (= min) -> Magenta (= max)
            cloud_it->r = value;
            cloud_it->g = 255 - value;
            cloud_it->b = value;
            break;
        case 2:
            // White (= min) -> Red (= max)
            cloud_it->r = 255;
            cloud_it->g = 255 - value;
            cloud_it->b = 255 - value;
            break;
        case 3:
            // Grey (< 128) / Red (> 128)
            if (value > 128)
            {
                cloud_it->r = 255;
                cloud_it->g = 0;
                cloud_it->b = 0;
            }
            else
            {
                cloud_it->r = 128;
                cloud_it->g = 128;
                cloud_it->b = 128;
            }
            break;
        default:
            // Blue -> Green -> Red (~ rainbow)
            cloud_it->r = value > 128 ? (value - 128) * 2 : 0;                 // r[128] = 0, r[255] = 255
            cloud_it->g = value < 128 ? 2 * value : 255 - ((value - 128) * 2); // g[0] = 0, g[128] = 255, g[255] = 0
            cloud_it->b = value < 128 ? 255 - (2 * value) : 0;                 // b[0] = 255, b[128] = 0
        }
    }
}
