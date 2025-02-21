#pragma once

#include "declarations.hpp"
#include "Material.hpp"
#include "PDCELFace.hpp"
#include "PDCELVertex.hpp"
#include "PModel.hpp"
#include "PSegment.hpp"
#include "PBaseLine.hpp"
#include "utilities.hpp"

#include <list>
#include <string>
#include <vector>

class Baseline;
class Material;
class PDCELFace;
class PDCELVertex;
class PModel;
class Segment;

/** @ingroup cs
 * A cross-sectional component class.
 */
class PComponent {
private:
  PModel *_pmodel;
  std::string _name;
  int _order; // The number deciding the building order
  int _type;  // 0: reserved; 1: laminate; 2: fill
  std::list<PComponent *> _dependencies;
  double _mesh_size = -1;
  std::vector<PDCELVertex *> _embedded_vertices;

  // Laminate type
  PDCELVertex *_ref_vertex = nullptr;
  std::vector<Segment *> _segments;
  int _style; // Join style. 1: step-like; 2: non-step-like
  bool _cycle;
  std::vector<int> _joint_styles;
  std::vector<std::vector<std::string>> _joint_segments;
  std::string _mat_orient_e1{"normal"};
  std::string _mat_orient_e2{"baseline"};

  // Filling type
  Material *_fill_material;
  LayerType *_fill_layertype;
  PDCELVertex *_fill_location;
  std::list<std::list<Baseline *>> _fill_baseline_groups;
  int _fill_side;
  Baseline *_fill_ref_baseline;
  PDCELFace *_fill_face;
  double _fill_theta1, _fill_theta3;

  // Trim
  bool _trim_head{false};
  bool _trim_tail{false};
  std::vector<double> _trim_head_vector;
  std::vector<double> _trim_tail_vector;

  // Helper functions
  // void findFreeEndVertices(Segment *seg, const int &end,
  //                          std::vector<PDCELVertex *> &vts);
  // void calcBoundingVertices(Segment *s, PDCELVertex *v);
  // void calcBoundingVertices(Segment *s1, PDCELVertex *v, Segment *s2);

  void createSegmentFreeEnd(Segment *s, int e, Message *);
  void joinSegments(Segment *s, int e, PDCELVertex *v, Message *);
  void joinSegments(Segment *s1, Segment *s2, int e1, int e2, PDCELVertex *v, int style, Message *);
  void buildLaminate(Message *);
  void buildFilling(Message *);

public:
  static int count_tmp;

  PComponent()
      : _order(0), _type(0), _style(1), _fill_location(nullptr),
        _fill_ref_baseline(nullptr){};
  PComponent(PModel *model)
      : _pmodel(model), _order(0), _type(0), _style(1), _cycle(false),
        _fill_location(nullptr), _fill_ref_baseline(nullptr){};
  PComponent(std::string name, PModel *model)
      : _name(name), _pmodel(model), _order(0), _type(0), _style(1),
        _cycle(false), _fill_location(nullptr), _fill_ref_baseline(nullptr){};

  void print();
  void print(Message *, int, int = 0);

  std::string name() { return _name; }
  std::vector<Segment *> segments() { return _segments; }
  int order();
  int type() { return _type; }
  int style() { return _style; }
  PModel *pmodel() { return _pmodel; }
  bool isCyclic() { return _cycle; }
  std::list<PComponent *> &dependents() { return _dependencies; }
  double getMeshSize() const { return _mesh_size; }
  std::vector<PDCELVertex *> getEmbeddedVertices() { return _embedded_vertices; }

  PDCELVertex *refVertex() { return _ref_vertex; }

  std::string getMatOrient1() { return _mat_orient_e1; }
  std::string getMatOrient2() { return _mat_orient_e2; }

  Material *material() { return _fill_material; }
  LayerType *filllayertype() { return _fill_layertype; }
  PDCELVertex *location() { return _fill_location; }
  std::list<std::list<Baseline *>> baselines() { return _fill_baseline_groups; }
  int fillside() { return _fill_side; }
  Baseline *refbaseline() { return _fill_ref_baseline; }
  PDCELFace *fillface() { return _fill_face; }
  double fillTheta1() { return _fill_theta1; }
  double fillTheta3() { return _fill_theta3; }

  bool isTrimHead() { return _trim_head; }
  bool isTrimTail() { return _trim_tail; }
  std::vector<double> getTrimHeadVector() { return _trim_head_vector; }
  std::vector<double> getTrimTailVector() { return _trim_tail_vector; }

  void setName(std::string);
  void addSegment(Segment *);
  void setOrder(int);
  void setType(int t) { _type = t; }
  void setStyle(int style) { _style = style; }
  void addDependent(PComponent *);
  void setMeshSize(double ms) { _mesh_size = ms; }
  void addEmbeddedVertex(PDCELVertex *v) { _embedded_vertices.push_back(v); }

  void setRefVertex(PDCELVertex *v) { _ref_vertex = v; }

  void setMatOrient1(std::string orient) { _mat_orient_e1 = orient; }
  void setMatOrient2(std::string orient) { _mat_orient_e2 = orient; }

  void setFillMaterial(Material *m) { _fill_material = m; }
  void setFillLayertype(LayerType *l) { _fill_layertype = l; }
  void setFillLocation(PDCELVertex *v) { _fill_location = v; }
  void addFillBaselineGroup(std::list<Baseline *> bg) {
    _fill_baseline_groups.push_back(bg);
  }
  void setFillSide(int s) { _fill_side = s; }
  void setFillRefBaseline(Baseline *b) { _fill_ref_baseline = b; }
  void setFillFace(PDCELFace *f) { _fill_face = f; }
  void setFillTheta1(double a) { _fill_theta1 = a; }
  void setFillTheta3(double a) { _fill_theta3 = a; }

  void addJointStyle(int s) { _joint_styles.push_back(s); }
  void addJointSegments(std::vector<std::string> sns) { _joint_segments.push_back(sns); }

  void setTrimHead(bool t) { _trim_head = t; }
  void setTrimTail(bool t) { _trim_tail = t; }
  void setTrimHeadVector(double x2, double x3) { _trim_head_vector.assign({x2, x3}); }
  void setTrimTailVector(double x2, double x3) { _trim_tail_vector.assign({x2, x3}); }

  void build(Message *);
  void buildDetails(Message *);
};

bool compareOrder(PComponent *, PComponent *);
