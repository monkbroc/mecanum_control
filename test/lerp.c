#include <stdio.h>

int linearInterpolate(int x, int x1, int x2, int y1, int y2) {
  if (x < x1) {
    return y1;
  }
  if (x > x2) {
    return y2;
  }
  return (x - x1) * (y2 - y1) / (x2 - x1) + y1;
}

/*
 2D linear interpolation
 The parameters are:
 z11 is at x1, y1
 z12 is at x1, y2
 z22 is at x2, y2
 z21 is at x2, y1

 It calculates the linear interpolation on the x axis at y1, the interpolation on the x axis at y2, then interpolates between these 2
*/
int linearInterpolate2D(int x, int y, int x1, int x2, int y1, int y2, int z11, int z12, int z22, int z21) {
  int zy1 = linearInterpolate(x, x1, x2, z11, z21);
  int zy2 = linearInterpolate(x, x1, x2, z12, z22);
  return linearInterpolate(y, y1, y2, zy1, zy2);
}

int main() {
    // printf("%d\n", linearInterpolate(50, 0, 100, 0, 1000));
    // printf("%d\n", linearInterpolate(-100, -150, -50, 100, 3000));
    printf("%d\n", linearInterpolate2D(40, 40, 0, 100, 0, 100, 500, 1500, 1500, 1500));
    return 0;
}