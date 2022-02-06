import java.util.Locale;

FlowField flow_field;
PVector target_pos;
float importance;
boolean locked = true;
boolean show_wonder = false;
boolean show_field = false;
KDTree sorted;
Path path;
Flock flock;

void setup() {
  //size(960, 540);
  size(640, 360);
  // Add an initial set of boids into the system
  //boid = new Boid(320, 180, 4);
  flow_field = new FlowField(20, 15);
  target_pos = new PVector(width/2, height/2);
  importance = 10;
  flock = new Flock(2);
  for(int i = 0; i < 400; i++){
    flock.add();
  }
  flock.setField(flow_field);
  //path = new Path();
  //path.addPoint(0, 0);
  //path.addPoint(140, 45);
  //path.addPoint(400, 260);
  //path.addPoint(640, 115);
  sorted = new KDTree(flock.boids);
}

void mousePressed(){
  importance = 10;
  locked = false;
  show_field = !show_field;
  target_pos = new PVector(mouseX, mouseY);
}

void mouseReleased(){
 locked = true; 
}

void draw() {
  background(50);
  sorted.regenerate();
  //if(!locked) importance += 1;
  //ellipse(target_pos.x, target_pos.y, importance, importance);
  //boid.findTarget(target_pos);
  //if(show_wonder){
  //  PVector pp = PVector.add(boid.position, PVector.mult(boid.velocity, boid.targ_radius*2));
  //  line(boid.position.x, boid.position.y, pp.x, pp.y);
  //  line(pp.x, pp.y, boid.wonder_targ.x, boid.wonder_targ.y);
  //  fill(0,0);
  //  ellipse(pp.x, pp.y, boid.targ_radius*2, boid.targ_radius*2);
  //  fill(200, 100);
  //  ellipse(boid.wonder_targ.x, boid.wonder_targ.y, 10, 10);
  //}
  //boid.fieldLookup(flow_field);
  //boid.findPath(path);
  //boid.update();
  flow_field.update();
  //flock.fieldLookup(flow_field);
  //flock.findPath(path);
  //boid.draw();
  //path.draw();
  flock.draw();
  if(show_field) flow_field.draw();
  noStroke();
  fill(255,200);
  rect(0,0, 60, 20);
  fill(0);
  text("fps: "+String.format(Locale.ENGLISH, "%5.2f", frameRate), 5, 15);
}
