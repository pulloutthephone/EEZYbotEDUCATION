void MAIN()
{
  // write your code here
  MoveL(p_ola, 0.050);
  MoveL(p1, 0.050);
  float* p1_novo = p_ola;
  p1_novo[0] = p1_novo[0] - 0.05;
  MoveL(p1_novo, 0.050);
  test();
}


void test() {

}