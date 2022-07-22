# no_detection_area_filter

## Purpose

The `no_detection_area_filter` is a node that removes points inside the area tagged as `no_detection_area` in lanelet2 map.

## Inner-workings / Algorithms
- Create the 2D polygon from the lanelet polygon that has the type of `no_detection_area`
- Remove input points in the polygon by `boost::geometry::within()`

![no_detection_area_figure](./image/no_detection_area_filter-overview.svg)


## Inputs / Outputs

### Input

| Name             | Type                                             | Description      |
| ---------------- | ------------------------------------------------ | ---------------- |
| `~/input/points` | `sensor_msgs::msg::PointCloud2`                  | input points |
| `~/input/vector_map` | `autoware_auto_mapping_msgs::msg::HADMapBin` | lanelet2 map used for filtering points      |

### Output

| Name              | Type                            | Description     |
| ----------------- | ------------------------------- | --------------- |
| `~/output/points` | `sensor_msgs::msg::PointCloud2` | filtered points |

## Parameters

### Core Parameters

## Assumptions / Known limits
