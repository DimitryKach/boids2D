class Path{
  ArrayList<PVector> points;
  float radius;
  
  Path(){
    radius = 20;
    points = new ArrayList<PVector>();
  }
  
  void addPoint(float x, float y){
    PVector point = new PVector(x, y);
    points.add(point);
  }
  
  void draw(){
    stroke(0);
    noFill();
    beginShape();
    
    for (PVector point : points){
      vertex(point.x, point.y);
    }
    endShape();
  }
}
