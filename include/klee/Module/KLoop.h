#ifndef KLOOP_H
#define KLOOP_H

#include "llvm/Analysis/LoopInfo.h"

#include <vector>

namespace klee {

  struct KLoop {
  public:
    llvm::Loop* loop;
    bool isSupported;

    KLoop(llvm::Loop *loop, bool isSupported) :
      loop(loop), isSupported(isSupported) {

    }

    KLoop() : loop(nullptr), isSupported(true) {

    }

    KLoop(const KLoop &other) : loop(other.loop), isSupported(other.isSupported) {

    }

  };

}

#endif /* KLOOP_H */
