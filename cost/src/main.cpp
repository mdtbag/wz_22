#include "app.h"

int main() {
  // 交互式菜单：不要用 sync_with_stdio(false) 之类“优化”，避免输出不刷新坑
  App app;
  app.run();
  return 0;
}
