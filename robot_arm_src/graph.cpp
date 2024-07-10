#include "graph.hpp"
#include <boost/graph/detail/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>
#include <fstream>
#include <iostream>

void loadRoadMap(roadmap_t &roadmap, transitionG_t &transitionG,
                 map<string, vertex_t> &vNameToV, map<string, edge_t> &eNameToE,
                 map<string, mode_tt> &mNameToMode,
                 map<string, primitive_tt> &pNameToPrimitive,
                 const map<string, int> &oNametoO, const string &roadmapFile) {

  std::ifstream problem_json(roadmapFile);
  assert(problem_json.good());
  json config;
  problem_json >> config;

  // Adding vertices
  const auto &vertices = config["vertices"];

  for (const auto &node : vertices) {
    string name = node["name"];

    auto mode_template = node["mode_template"];
    string mode_type = mode_template[0];
    string mode_element = mode_template[1];
    string mode_object = mode_template[2];
    string mode_parameter = mode_template[3];
    string mode_string = "[" + mode_type + ", " + mode_element + ", " +
                         mode_object + ", " + mode_parameter + "]";

    auto iter = mNameToMode.find(mode_string);
    mode_tt mode;
    if (iter == mNameToMode.end()) {
      mode = boost::add_vertex(transitionG);
      transitionG[mode].name = mode_string;
      transitionG[mode].type = mode_type;
      if (mode_object == "NA") {
        transitionG[mode].object_id = -1;
      } else {
        transitionG[mode].object_id = oNametoO.find(mode_object)->second;
      }
      transitionG[mode].parameter = mode_parameter;
      mNameToMode[mode_string] = mode;
    } else {
      mode = mNameToMode[mode_string];
    }
    auto vertex = boost::add_vertex(roadmap);
    roadmap[vertex].name = name;
    roadmap[vertex].mode = mode;
    vNameToV[name] = vertex;
  }

  // Adding edges
  const auto &edges = config["edges"];
  for (const auto &node : edges) {
    string name = node["name"];
    string fromName = node["from"];
    string toName = node["to"];
    auto duration = node["duration"];
    auto fromVertexGID = vNameToV.find(fromName)->second;
    auto toVertexGID = vNameToV.find(toName)->second;
    auto fromModeGID = roadmap[fromVertexGID].mode;
    auto toModeGID = roadmap[toVertexGID].mode;
    auto primitive_list = node["primitive_template"];
    string primitive_type = primitive_list[0];
    string primitive_element = primitive_list[1];
    string primitive_object = primitive_list[2];
    string primitive_parameter = primitive_list[3];
    string primitive_string = "[" + primitive_type + ", " + primitive_element +
                              ", " + primitive_object + ", " +
                              primitive_parameter + "]";
    primitive_tt primitive;
    auto iter = pNameToPrimitive.find(primitive_string);
    if (iter == pNameToPrimitive.end()) {
      primitive = boost::add_edge(fromModeGID, toModeGID, transitionG).first;
      transitionG[primitive].name = primitive_string;
      transitionG[primitive].type = primitive_type;
      if (primitive_object == "NA") {
        transitionG[primitive].object_id = -1;
      } else {
        transitionG[primitive].object_id =
            oNametoO.find(primitive_object)->second;
      }
      transitionG[primitive].parameter = primitive_parameter;
      pNameToPrimitive[primitive_string] = primitive;
    } else {
      primitive = pNameToPrimitive[primitive_string];
    }

    auto edge = boost::add_edge(fromVertexGID, toVertexGID, roadmap);
    roadmap[edge.first].name = name;
    roadmap[edge.first].duration = duration;
    roadmap[edge.first].primitive = primitive;
    eNameToE[name] = edge.first;
  }
}

void printRoadMap(const roadmap_t &roadmap, const transitionG_t &transitionG,
                  const map<string, vertex_t> &vNameToV,
                  const map<string, edge_t> &eNameToE) {

  std::cout << "\tRoadmap: " << std::endl;
  std::cout << "\t|V| = " << boost::num_vertices(roadmap)
            << ", |E| = " << boost::num_edges(roadmap) << std::endl
            << "\t\t";
  for (auto &v : roadmap.m_vertices) {
    auto modeGID = v.m_property.mode;
    auto mode = transitionG[modeGID];
    std::cout << vNameToV.at(v.m_property.name) << ": " << v.m_property.name
              << ", [" << mode.type << "," << mode.object_id << ","
              << mode.parameter << "] ,";
  }

  std::cout << std::endl << "\t\t";
  boost::graph_traits<roadmap_t>::edge_iterator ei, ei_end;
  for (tie(ei, ei_end) = boost::edges(roadmap); ei != ei_end; ++ei) {
    auto primitiveGID = roadmap[*ei].primitive;
    auto primitive = transitionG[primitiveGID];
    std::cout << eNameToE.at(roadmap[*ei].name) << ": " << roadmap[*ei].name
              << ", " << roadmap[*ei].duration << ", [" << primitive.type << ","
              << primitive.object_id << "," << primitive.parameter << "] ,";
  }
  std::cout << std::endl;
}
