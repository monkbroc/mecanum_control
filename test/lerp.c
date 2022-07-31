#include <stdio.h>

int linearInterpolate(int val, int x0, int x1, int y0, int y1) {
  if (val < x0) {
    return y0;
  }
  if (val > x1) {
    return y1;
  }
  return (val - x0) * (y1 - y0) / (x1 - x0) + y0;
}

int main() {
    printf("%d\n", linearInterpolate(50, 0, 100, 0, 1000));
    printf("%d\n", linearInterpolate(-100, -150, -50, 100, 3000));
    return 0;
}