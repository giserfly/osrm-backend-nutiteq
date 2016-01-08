/*

Copyright (c) 2015, Project OSRM contributors
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.
Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef NUTI_VIA_ROUTE_HPP
#define NUTI_VIA_ROUTE_HPP

#include "../plugins/plugin_base.hpp"

#include "../algorithms/douglas_peucker.hpp"
#include "../algorithms/object_encoder.hpp"
#include "../data_structures/search_engine.hpp"
#include "../descriptors/descriptor_base.hpp"
#include "../descriptors/gpx_descriptor.hpp"
#include "../descriptors/json_descriptor.hpp"
#include "../util/integer_range.hpp"
#include "../util/json_renderer.hpp"
#include "../util/make_unique.hpp"
#include "../util/simple_logger.hpp"
#include "../util/timing_util.hpp"
#include "../algorithms/polyline_formatter.hpp"

#include "../nutiteq/engine/Routing/RoutingObjects.h"
#include "../nutiteq/engine/Routing/RoutingGraph.h"
#include "../nutiteq/engine/Routing/RouteFinder.h"

#include <osrm/json_container.hpp>

#include <cstdlib>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>
#include <set>

#include <boost/filesystem.hpp>

class NutiViaRoutePlugin final : public BasePlugin
{
  private:
    DescriptorTable descriptor_table;
    std::string descriptor_string;
    DouglasPeucker polyline_generalizer;
    std::shared_ptr<Nuti::Routing::RoutingGraph> routing_graph;
    int max_locations_viaroute;

  public:
    explicit NutiViaRoutePlugin(const boost::filesystem::path& base_path, int max_locations_viaroute)
        : descriptor_string("viaroute"),
          max_locations_viaroute(max_locations_viaroute)
    {
        namespace fs = boost::filesystem;

        Nuti::Routing::RoutingGraph::Settings graph_settings;
        graph_settings.nodeBlockCacheSize = 512 * 16;
        graph_settings.geometryBlockCacheSize = 512 * 16;
        graph_settings.nameBlockCacheSize = 64 * 16;
        graph_settings.globalNodeBlockCacheSize = 64 * 16;
        graph_settings.rtreeNodeBlockCacheSize = 16 * 16;
        routing_graph = std::make_shared<Nuti::Routing::RoutingGraph>(graph_settings);

        fs::directory_iterator end_iter;
        std::set<std::string> nutigraph_files;
        for (fs::directory_iterator dir_iter(base_path); dir_iter != end_iter; ++dir_iter)
        {
            std::string file_name = dir_iter->path().string();
            if (fs::is_regular_file(dir_iter->status()) && file_name.rfind(".nutigraph") == file_name.size() - 10)
            {
                nutigraph_files.insert(file_name.substr(0, file_name.size() - 10));
            }
        }
        
        for (const std::string& nutigraph_file : nutigraph_files)
        {
            std::string::size_type pos = nutigraph_file.find('-');
            if (pos != std::string::npos)
            {
                std::string parent_nutigraph_file = nutigraph_file.substr(0, pos);
                if (nutigraph_files.count(parent_nutigraph_file) > 0)
                {
                    SimpleLogger().Write(logINFO) << "Skipping " << (nutigraph_file + ".nutigraph") << " as " << (parent_nutigraph_file + ".nutigraph") << " exists";
                    continue;
                }
            }
            try
            {
                SimpleLogger().Write(logINFO) << "Loading " << (nutigraph_file + ".nutigraph");
                routing_graph->import(nutigraph_file + ".nutigraph");
            }
            catch (const std::exception& ex)
            {
                SimpleLogger().Write(logWARNING) << "Failed to load " << (nutigraph_file + ".nutigraph") << ": " << ex.what();
            }
        }
        
        descriptor_table.emplace("json", 0);
    }

    virtual ~NutiViaRoutePlugin() {}

    const std::string GetDescriptor() const override final { return descriptor_string; }

    Status HandleRequest(const RouteParameters &route_parameters,
                      osrm::json::Object &json_result) override final
    {
        if (max_locations_viaroute > 0 &&
            (static_cast<int>(route_parameters.coordinates.size()) > max_locations_viaroute))
        {
            json_result.values["status_message"] =
                "Number of entries " + std::to_string(route_parameters.coordinates.size()) +
                " is higher than current maximum (" + std::to_string(max_locations_viaroute) + ")";
            return Status::Error;
        }

        if (route_parameters.coordinates.size() < 2)
        {
            json_result.values["status_message"] = "Invalid coordinates";
            return Status::Error;
        }

        std::vector<Nuti::Routing::RoutingResult> results;
        for (std::size_t i = 1; i < route_parameters.coordinates.size(); i++)
        {
            Nuti::Routing::WGSPos pos0(route_parameters.coordinates[0].lat / COORDINATE_PRECISION, route_parameters.coordinates[0].lon / COORDINATE_PRECISION);
            Nuti::Routing::WGSPos pos1(route_parameters.coordinates[1].lat / COORDINATE_PRECISION, route_parameters.coordinates[1].lon / COORDINATE_PRECISION);
            Nuti::Routing::RouteFinder finder(routing_graph);
            Nuti::Routing::RoutingResult result;
            try
            {
                result = finder.find(Nuti::Routing::RoutingQuery(pos0, pos1));
            }
            catch (const std::exception& ex)
            {
                json_result.values["status_message"] = std::string("Routing failed, exception: ") + ex.what();
                return Status::Error;
            }
            if (result.getStatus() == Nuti::Routing::RoutingResult::Status::FAILED)
            {
                json_result.values["status_message"] = "Routing failed";
                return Status::Error;
            }
            results.push_back(std::move(result));
        }

        std::vector<SegmentInformation> path_description;
        osrm::json::Array json_route_instructions;

        for (size_t i = 0; i < results.size(); i++)
        {
            const Nuti::Routing::RoutingResult& result = results[i];
            if (result.getInstructions().empty())
            {
                continue;
            }

            std::size_t path_index = path_description.size();
            for (const Nuti::Routing::WGSPos& pos : result.getGeometry())
            {
                FixedPointCoordinate segment_pos(static_cast<int>(pos(0) * COORDINATE_PRECISION), static_cast<int>(pos(1) * COORDINATE_PRECISION));
                SegmentInformation segment(segment_pos, 0, 0, 0, TurnInstruction::NoTurn, true, true, TRAVEL_MODE_INACCESSIBLE);
                path_description.push_back(segment);
            }
            
            for (std::size_t i = std::max(static_cast<std::size_t>(1), path_index); i < path_description.size(); i++)
            {
                SegmentInformation& first = path_description[i - 1];
                SegmentInformation& second = path_description[i];
                const double post_turn_bearing = coordinate_calculation::bearing(first.location, second.location);
                const double pre_turn_bearing = coordinate_calculation::bearing(second.location, first.location);
                second.post_turn_bearing = static_cast<short>(post_turn_bearing * 10);
                second.pre_turn_bearing = static_cast<short>(pre_turn_bearing * 10);
            }

            double distance = 0;
            double time = 0;
            for (const Nuti::Routing::RoutingInstruction& instr : result.getInstructions())
            {
                distance += instr.getDistance();
                time += instr.getTime();

                Nuti::Routing::RoutingInstruction::Type type = instr.getType();
                if (type == Nuti::Routing::RoutingInstruction::Type::NO_TURN || type == Nuti::Routing::RoutingInstruction::Type::STAY_ON_ROUNDABOUT)
                {
                    continue;
                }
                if (type == Nuti::Routing::RoutingInstruction::Type::REACHED_YOUR_DESTINATION && i + 1 < results.size())
                {
                    type = Nuti::Routing::RoutingInstruction::Type::REACH_VIA_LOCATION;
                }

                std::size_t point_index = path_index + instr.getGeometryIndex();

                osrm::json::Array json_instruction_row;
                json_instruction_row.values.push_back(std::to_string(static_cast<int>(type)));
                json_instruction_row.values.push_back(instr.getAddress());
                json_instruction_row.values.push_back(distance);
                json_instruction_row.values.push_back(point_index);
                json_instruction_row.values.push_back(time);
                json_instruction_row.values.push_back(std::to_string(static_cast<unsigned>(distance)) + "m");

                const double post_turn_bearing_value = (path_description[point_index].post_turn_bearing / 10.0);
                json_instruction_row.values.push_back(bearing::get(post_turn_bearing_value));
                json_instruction_row.values.push_back(post_turn_bearing_value);

                const double pre_turn_bearing_value = (path_description[point_index].pre_turn_bearing / 10.0);
                json_instruction_row.values.push_back(bearing::get(pre_turn_bearing_value));
                json_instruction_row.values.push_back(pre_turn_bearing_value);

                json_route_instructions.values.push_back(json_instruction_row);

                distance = 0;
                time = 0;
            }
        }

        // Generalize poly line
        polyline_generalizer.Run(path_description.begin(), path_description.end(), route_parameters.zoom_level);

        json_result.values["route_geometry"] = PolylineFormatter().printEncodedString(path_description);
        json_result.values["status_message"] = "Found route between points";
        json_result.values["route_instructions"] = json_route_instructions;

#if 0

        if (!check_all_coordinates(route_parameters.coordinates))
        {
            json_result.values["status_message"] = "Invalid coordinates";
            return Status::Error;
        }

        const auto &input_bearings = route_parameters.bearings;
        if (input_bearings.size() > 0 &&
            route_parameters.coordinates.size() != input_bearings.size())
        {
            json_result.values["status_message"] =
                "Number of bearings does not match number of coordinate";
            return Status::Error;
        }

        std::vector<PhantomNodePair> phantom_node_pair_list(route_parameters.coordinates.size());
        const bool checksum_OK = (route_parameters.check_sum == facade->GetCheckSum());



        for (const auto i : osrm::irange<std::size_t>(0, route_parameters.coordinates.size()))
        {
            if (checksum_OK && i < route_parameters.hints.size() &&
                !route_parameters.hints[i].empty())
            {
                ObjectEncoder::DecodeFromBase64(route_parameters.hints[i],
                                                phantom_node_pair_list[i].first);
                if (phantom_node_pair_list[i].first.is_valid(facade->GetNumberOfNodes()))
                {
                    continue;
                }
            }
            const int bearing = input_bearings.size() > 0 ? input_bearings[i].first : 0;
            const int range = input_bearings.size() > 0
                                  ? (input_bearings[i].second ? *input_bearings[i].second : 10)
                                  : 180;
            phantom_node_pair_list[i] = facade->NearestPhantomNodeWithAlternativeFromBigComponent(
                route_parameters.coordinates[i], bearing, range);
            // we didn't found a fitting node, return error
            if (!phantom_node_pair_list[i].first.is_valid(facade->GetNumberOfNodes()))
            {
                json_result.values["status_message"] =
                    std::string("Could not find a matching segment for coordinate ") +
                    std::to_string(i);
                return Status::NoSegment;
            }
            BOOST_ASSERT(phantom_node_pair_list[i].first.is_valid(facade->GetNumberOfNodes()));
            BOOST_ASSERT(phantom_node_pair_list[i].second.is_valid(facade->GetNumberOfNodes()));
        }

        auto snapped_phantoms = snapPhantomNodes(phantom_node_pair_list);

        InternalRouteResult raw_route;
        auto build_phantom_pairs = [&raw_route](const PhantomNode &first_node,
                                                const PhantomNode &second_node)
        {
            raw_route.segment_end_coordinates.push_back(PhantomNodes{first_node, second_node});
        };
        osrm::for_each_pair(snapped_phantoms, build_phantom_pairs);

        if (1 == raw_route.segment_end_coordinates.size())
        {
            if (route_parameters.alternate_route)
            {
                search_engine_ptr->alternative_path(raw_route.segment_end_coordinates.front(),
                                                    raw_route);
            }
            else
            {
                search_engine_ptr->direct_shortest_path(raw_route.segment_end_coordinates,
                                                        route_parameters.uturns, raw_route);
            }
        }
        else
        {
            search_engine_ptr->shortest_path(raw_route.segment_end_coordinates,
                                             route_parameters.uturns, raw_route);
        }

        bool no_route = INVALID_EDGE_WEIGHT == raw_route.shortest_path_length;

        std::unique_ptr<BaseDescriptor<DataFacadeT>> descriptor;
        switch (descriptor_table.get_id(route_parameters.output_format))
        {
        case 1:
            descriptor = osrm::make_unique<GPXDescriptor<DataFacadeT>>(facade);
            break;
        // case 2:
        //      descriptor = osrm::make_unique<GEOJSONDescriptor<DataFacadeT>>();
        //      break;
        default:
            descriptor = osrm::make_unique<JSONDescriptor<DataFacadeT>>(facade);
            break;
        }

        descriptor->SetConfig(route_parameters);
        descriptor->Run(raw_route, json_result);

        // we can only know this after the fact, different SCC ids still
        // allow for connection in one direction.
        if (no_route)
        {
            auto first_component_id = snapped_phantoms.front().component.id;
            auto not_in_same_component =
                std::any_of(snapped_phantoms.begin(), snapped_phantoms.end(),
                            [first_component_id](const PhantomNode &node)
                            {
                                return node.component.id != first_component_id;
                            });
            if (not_in_same_component)
            {
                json_result.values["status_message"] = "Impossible route between points";
                return Status::EmptyResult;
            }
        }
        else
        {
            json_result.values["status_message"] = "Found route between points";
        }

#endif
        return Status::Ok;


    }
};

#endif // VIA_ROUTE_HPP
