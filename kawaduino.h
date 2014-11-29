class Color {
public:
  Color() {};
  Color(int r, int g, int b) : r(r), g(g), b(b) {};
  int r, g, b;
  uint32_t toUint32() { return ((uint32_t)r << 16) | ((uint32_t)g <<  8) | b; };
};
