class FlowField{
  PVector[][] vectors;
  float[] rots;
  int cols, rows;
  int res;
  float vec_scale;
  
  FlowField(int num, float s){
    res = num;
    cols = width/res;
    rows = height/res;
    vec_scale = s;
    vectors = new PVector[cols][rows];
    rots = new float[cols*rows];
    initField();
  }
  
  void initField(){
   float xoff = 0;
   for (int c = 0; c < cols; c++){
     float yoff = 0;
      for (int r = 0; r < rows; r++){
        float theta = map(noise(xoff,yoff),0,1,0,TWO_PI);
        float theta_r = map(noise(xoff,yoff),0,1,0,PI/4);
        vectors[c][r] = new PVector(cos(theta), sin(theta));
        float sign = random(-1, 1);
        if (sign <= 0) sign = -1;
        if (sign > 0) sign = 1;
        rots[c*rows + r] = theta_r * sign;
        yoff += 0.1;
      }
      xoff += 0.1;
    }
  }
  
  PVector lookup(PVector lookup_pos){
    int column = int(constrain(lookup_pos.x/res, 0, cols-1));
    int row = int(constrain(lookup_pos.y/res, 0, rows-1));
    return vectors[column][row];
  }
  
  void update(){
    for (int c = 0; c < cols; c++){
      for (int r = 0; r < rows; r++){
        vectors[c][r].rotate(rots[c*rows + r]*0.1).normalize();
        //float rand = random(-0.2, 0.2);
        //vectors[c][r].add(new PVector(rand, rand, rand));
      }
    }
  }
  
  void draw(){
    float x_shift = float(width)/cols;
    float y_shift = float(height)/rows;
    PVector shift_vec = new PVector(x_shift, y_shift);
    for (int c = 0; c < cols; c++){
      for (int r = 0; r < rows; r++){
        PVector cur_pos = new PVector(shift_vec.x*c+(x_shift/2), shift_vec.y*r+(y_shift/2));
        //println(cur_pos);
        drawArrow(cur_pos, vectors[c][r], vec_scale);
      }
    }
  }
  
  void drawArrow(PVector pos, PVector dir, float scale){
    PVector n_pos = PVector.add(pos, PVector.mult(PVector.mult(dir, -1), scale/2));
    PVector end = PVector.add(n_pos, PVector.mult(dir, scale));
    PVector r_tip = PVector.sub(end, n_pos).normalize();
    r_tip = new PVector(cos(PI/4)*r_tip.x - sin(PI/4)*r_tip.y, sin(PI/4)*r_tip.x + cos(PI/4)*r_tip.y);
    r_tip.mult(scale/3);
    r_tip = PVector.sub(end, r_tip);
    PVector l_tip = PVector.sub(end, n_pos).normalize();
    l_tip = new PVector(cos(7*PI/4)*l_tip.x - sin(7*PI/4)*l_tip.y, sin(7*PI/4)*l_tip.x + cos(7*PI/4)*l_tip.y);
    l_tip.mult(scale/3);
    l_tip = PVector.sub(end, l_tip);
    // Draw
    stroke(255);
    line(n_pos.x, n_pos.y, end.x, end.y);
    line(end.x, end.y, r_tip.x, r_tip.y);
    line(end.x, end.y, l_tip.x, l_tip.y);
  }
}
