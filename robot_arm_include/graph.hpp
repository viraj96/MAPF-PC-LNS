#pragma once

#include <boost/graph/adjacency_list.hpp>
#include <nlohmann/json.hpp>
#include <string>

using std::map;
using std::string;
using json = nlohmann::json;

typedef boost::adjacency_list_traits<boost::vecS, boost::vecS, boost::directedS>
    GraphTraits_t;

typedef GraphTraits_t::edge_descriptor edge_t;
typedef GraphTraits_t::vertex_descriptor vertex_t;

typedef GraphTraits_t::vertex_descriptor mode_tt;
typedef GraphTraits_t::edge_descriptor primitive_tt;

struct Vertex {
  string name;
  mode_tt mode;
};

struct Edge {
  string name;
  double duration;
  primitive_tt primitive;
};

struct Mode {
  string name;
  string type;
  int object_id;
  string parameter;
};

struct Primitive {
  string name;
  string type;
  int object_id;
  string parameter;
};

typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS,
                              Vertex, Edge>
    roadmap_t;
typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS, Mode,
                              Primitive>
    transitionG_t;

void loadRoadMap(roadmap_t &roadmap, transitionG_t &transitionG,
                 map<string, vertex_t> &vNameToV, map<string, edge_t> &eNameToE,
                 map<string, mode_tt> &mNameToMode,
                 map<string, primitive_tt> &pNameToPrimitive,
                 const map<string, int> &oNametoO, const string &roadmapFile);

void printRoadMap(const roadmap_t &roadmap, const transitionG_t &transitionG,
                  const map<string, vertex_t> &vNameToV,
                  const map<string, edge_t> &eNameToE);

template <class T>
boost::iterator_range<T> pair_range(std::pair<T, T> const &p) {
  return boost::make_iterator_range(p.first, p.second);
}
