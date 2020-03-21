/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "./render/render.h"

// Structure to represent node of kd tree
struct Node {
  std::vector<float> point;
  int id;
  Node *left;
  Node *right;

  Node(std::vector<float> arr, int setId)
      : point(arr), id(setId), left(NULL), right(NULL) {}
};

struct KdTree {
  Node *root;

  KdTree() : root(NULL) {}

  void helperInsert(Node *&node, std::vector<float> point, int id, int depth) {
    if (node == NULL) {
      node = new Node(point, id);
    } else {
      int dim = depth % 3;  // 0:x; 1:y
      if (point[dim] < node->point[dim]) {
        helperInsert(node->left, point, id, depth + 1);
      } else {
        helperInsert(node->right, point, id, depth + 1);
      }
    }
  }

  void insert(std::vector<float> point, int id) {
    // TODO: Fill in this function to insert a new point into the tree
    // the function should create a new node and place correctly with in the
    // root
    helperInsert(root, point, id, 0);
  }

  bool isInBox(std::vector<float> &point, std::vector<float> &target,
               float distanceTol) {
    if ((point[0] >= (target[0] - distanceTol)) &&
        (point[0] <= (target[0] + distanceTol)) &&
        (point[1] >= (target[1] - distanceTol)) &&
        (point[1] <= (target[1] + distanceTol)) &&
        (point[2] >= (target[2] - distanceTol)) &&
        (point[2] <= (target[2] + distanceTol))) {
      return true;
    } else {
      return false;
    }
  }

  void searchHelper(Node *&node, std::vector<int> &ids, int depth,
                    std::vector<float> &target, float distanceTol) {
    if (node == NULL) {
      return;
    } else {
      if (isInBox(node->point, target, distanceTol)) {
        float dx = node->point[0] - target[0];
        float dy = node->point[1] - target[1];
        float dz = node->point[2] - target[2];
        float d = sqrt(dx * dx + dy * dy + dz * dz);
        if (d <= distanceTol) {
          ids.push_back(node->id);
        }
      }
      int dim = depth % 3;
      if (node->point[dim] > target[dim] - distanceTol) {
        searchHelper(node->left, ids, depth + 1, target, distanceTol);
      }
      if (node->point[dim] < target[dim] + distanceTol) {
        searchHelper(node->right, ids, depth + 1, target, distanceTol);
      }
    }
  }
  // return a list of point ids in the tree that are within distance of target
  std::vector<int> search(std::vector<float> target, float distanceTol) {
    std::vector<int> ids;
    searchHelper(root, ids, 0, target, distanceTol);
    return ids;
  }
};
