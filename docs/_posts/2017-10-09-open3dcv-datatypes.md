---
layout: page
title: "Open3DCV Datatypes"
category: tut
order: 0
---

This tutorial introduces all open3DCV related data types. 

> This library is still under active development, thus the definitions may be constantly updated, this page may not be updated as constantly to reflect the latest versions.

### Overview
* [Keypoint type](#keypoint_type)
* [Keypoint](#keypoint)
* [Sift parameters](#sift_param)
* [DMatch](#dmatch)
* [Pair](#pair)
* [Track](#track)
* [Structure_Point](#struct_point)
* [Graph](#graph)

### Keypoint type <a name="keypoint_type"></a>
`Keypoint.h`

```cpp
enum KeypointType
{
    INVALID = -1,
    DOG = 0,
    HARRIS = 1,
    SIFT = 2,
    SURF = 3,
};
```

### Keypoint <a name="keypoint"></a>
`keypoint.h`

```cpp
class Keypoint
{
public:
    Keypoint();
    Keypoint(const Vec2f &r_x);
    Keypoint(const Vec2f &r_x, unsigned int r_i);
    Keypoint(const Vec2f &r_x, unsigned int r_i, const Vec3i &r_c);
    Keypoint(const Vec2f &r_x, const float r_s, const float r_o);
    Keypoint(const Keypoint& key);
    Keypoint& operator=(const Keypoint& key);
    virtual ~Keypoint() { };
    
    const Vec2f &coords() const;
    Vec2f& coords();
    const unsigned int& index() const;
    unsigned int& index();
    const Vec3i& color() const;
    Vec3i& color();
    const double& scale() const;
    double& scale();
    const int has_scale() const;
    const double& orientation() const;
    double& orientation();
    const int has_orientation() const;
    
    bool operator<(const Keypoint& rhs) const;
    friend std::ostream& operator<<(std::ostream& ostr, const Keypoint& rhs);
    friend std::istream& operator>>(std::istream& istr, Keypoint& rhs);
    
    static int is_identical(const Keypoint& key1, const Keypoint& key2);

private:
    // The difference between the implemented Keypoint and VLFeat is that the latter assumes that
    // the image origin (top-left corner) has coordinate (0,0) as opposed to (1,1)
    Vec2f coords_;                      // coordinates
    unsigned int index_;                // Image index
    Vec3i color_;                       // color
    double scale_;                      // scale
    double orientation_;                // orientation
    std::pair<int, int> id_;            // id of the feature, <img_ind, feat_ind>, not used
    KeypointType keypoint_type_;        // keypoint type, not used
    
};
```

### Sift parameters <a name="sift_param"></a>
`sift_param.h`

```cpp
class SiftParam
{
public:
    SiftParam();
    SiftParam(int num_octaves,
              int num_levels,
              int first_octave,
              float edge_thresh,
              float peak_thresh,
              float norm_thresh,
              float magnif,
              float window_size);
    
    SiftParam(int num_octaves,
              int num_levels,
              int first_octave,
              float edge_thresh,
              float peak_thresh);
    
    SiftParam(int num_octaves,
              int num_levels,
              int first_octave);
    
    SiftParam(const SiftParam& SiftParam);
    
    ~SiftParam();
    
    /********** Detector parameters **********/
    // number of octave of the DoG scale space.
    int num_octaves_;
    // number of levels per octave of the DoG scale space.
    int num_levels_;
    // index of the first octave of the DoG scale space.
    int first_octave_;
    // peak selection threshold, decrease to eliminate more keypoints
    float edge_thresh_;
    // non-edge selection threshold, increase to eliminate more keypoints
    float peak_thresh_;
    
    /********** Descriptor parameters **********/
    // Set the minimum l2-norm of the descriptors before normalization.
    // Descriptors below the threshold are set to zero.
    float norm_thresh_;
    // Set the descriptor magnification factor. The scale of the
    // keypoint is multiplied by this factor to obtain the width (in
    // pixels) of the spatial bins. For instance, if there are 
    // 4 spatial bins along each spatial direction, the
    // ``side'' of the descriptor is approximatively 4 * MAGNIF.
    float magnif_;
    // Set the variance of the Gaussian window that determines the
    // descriptor support. It is expressend in units of spatial bins.
    float window_size_;
    
    bool root_sift_;
    /* Upright sift enables only a single descriptor to be extracted at a given
     * location. This is useful for SfM for a number of reasons, especially during
     * geometric verification.
     */
    bool upright_sift_;
};
```

### DMatch <a name="dmatch"></a>
`dmatch.h`

```cpp
class DMatch
{
public:
    DMatch () {};
    DMatch (const int r_ikey1, const int r_ikey2, const float dist) :
        ind_key_(r_ikey1, r_ikey2), dist_(dist) {};
    DMatch (const int r_ikey1, const int r_ikey2, const Vec2f r_pt1, const Vec2f r_pt2, const float dist) :
        ind_key_(r_ikey1, r_ikey2), point_(r_pt1, r_pt2), dist_(dist) {};
    DMatch (const DMatch& match) :
        ind_key_(match.ind_key_), point_(match.point_), dist_(match.dist_) {};
    
    void update_match_pt(const std::vector<Keypoint>& key1, const std::vector<Keypoint>& key2);
    const float& dist() const;
    
    std::pair<int, int> ind_key_;
    std::pair<Vec2f, Vec2f> point_;
    float dist_;
};
```

### Pair <a name="pair"></a>
`pair.h`

```cpp
class Pair
{
public:
    Pair();
    Pair(const int cam1, const int cam2);
    Pair(const int cam1, const int cam2, const std::vector<std::pair<Vec2f, Vec2f> >& matches);
    Pair(const int cam1, const int cam2, const std::vector<DMatch>& matches);
    ~Pair();
    
    void init(const int ind_cam1, const int ind_cam2);
    void update_matches(const int* vote_inlier);
    void update_intrinsics(const float f, const int w, const int h);
    bool operator<(const Pair& rhs) const;
    float baseline_angle() const;
    
    std::vector<int> cams_;
    std::vector<DMatch> matches_;
    Mat3f F_;
    Mat3f E_;
    std::vector<Mat3f> intrinsics_mat_;
    std::vector<Mat34f> extrinsics_mat_;
    
};
```

### Track <a name="track"></a>
`track.h`

```cpp
class Track
{
public:
    Track();
    Track(const Track& track);
    Track& operator=(const Track& track);
    virtual ~Track();
    
    int size() const; // length of the feature track
    const Keypoint &operator[](int index) const;
    void add_keypoint(const Keypoint &k);
    void rm_keypoint(int index);
    std::vector<Keypoint>::iterator key_begin();
    std::vector<Keypoint>::iterator key_end();
    static int has_overlapping_keypoints(const Track& track1, const Track& track2);
    static void find_overlapping_keypoints(const Track& track1, const Track& track2, std::vector<std::pair<int, int> >& ind_key);
    
protected:
    std::vector<Keypoint> keys_; // the collection of keypoint correspondences
    
};
```

### Structure_Point <a name="struct_point"></a>
`structure_point.h`

```cpp
class Structure_Point
{
public:
    Structure_Point();
    Structure_Point(const Vec3f &coord);
    Structure_Point(const Vec3f &coord, const Vec3i &color);
    Structure_Point& operator=(const Structure_Point& struct_pt);
    virtual ~Structure_Point();
    
    const Vec3f &coords() const;
    Vec3f &coords();
    const Vec3i &color() const;
    Vec3i &color();
    
protected:
    Vec3f p_; // The x,y,z coordinates, better is it's of double
    Vec3i c_; //!< The r,g,b color components (0-255 for each)
    
};
```

### Graph <a name="graph"></a>
`graph.h`

```cpp
class Graph
{
public:
    Graph();
    Graph(const Pair& pair);
    Graph(const Graph& graph);
    Graph& operator=(const Graph& graph);
    ~Graph();
    
    void init(const Graph& graph);
    void init(const Pair& pair);
    int index(int icam) const;
    int sz_cams() const; // number of cameras;
    int sz_tracks() const; // number of feature tracks;
    void add_track(const Track& track);
    void rm_track(int index);
    void add_struct_pt(const Structure_Point& struct_pt);
    void rm_struct_pt(int index);
    void rm_outliers(const float thresh_reproj, const float thresh_angle);
    bool operator<(const Graph& rhs) const;
    float baseline_angle() const;
    static void merge_graph(Graph& graph1, Graph& graph2);
    static void merge_tracks(Track& track1, const Track& track2, std::vector<std::pair<int, int> >& ind_key);
    static int find_next_graph(const std::vector<Graph>& graphs, const Graph& graph, std::vector<int>& merged_graph);
    static void report_graph(const Graph& graph);

    int ncams_;
    std::vector<int> cams_;
    std::vector<Mat3f> intrinsics_mat_;
    std::vector<Mat34f> extrinsics_mat_;
    std::vector<Track> tracks_;
    std::vector<Structure_Point> structure_points_;
};
```