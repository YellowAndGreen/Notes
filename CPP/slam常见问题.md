# slam常见问题

## Pangolin: error: ‘decay_t’ is not a member of ‘std’

解决方法：删除cmakelists中的`set(CMAKE_CXX_FLAGS "-std=c++11 -O2")`

 ## Ubuntu crashed and can't enter graphic mode after compiling openCV

solution: install ubuntu-desktop using aptitude

