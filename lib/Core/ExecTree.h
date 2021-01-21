#ifndef KLEE_EXEC_TREE_H
#define KLEE_EXEC_TREE_H

#include <klee/Expr/Expr.h>

#include <stdint.h>
#include <vector>

namespace klee {

class ExecTreeNode {
public:

  ExecTreeNode(std::uint32_t stateID, ref<Expr> e) :
    stateID(stateID), e(e), left(nullptr), right(nullptr) {

  }

  bool isLeaf() {
    return left == nullptr && right == nullptr;
  }

  std::uint32_t stateID;
  ref<Expr> e;
  ExecTreeNode *left;
  ExecTreeNode *right;
};

class ExecTree {
public:

  ExecTree(std::uint32_t stateID);

  ~ExecTree();

  void addNode(ExecTreeNode *node);

  void extend(std::uint32_t stateID,
              ref<Expr> condition,
              std::uint32_t leftID,
              std::uint32_t rightID);

  void dump();

  ExecTreeNode *root;
  std::vector<ExecTreeNode *> nodes;
};

}

#endif
