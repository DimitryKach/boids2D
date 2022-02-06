boolean USE_BOIDS = true;

class Flock{
  ArrayList<Boid> boids;
  float scale;
  
  Flock(float s){
    boids = new ArrayList<Boid>();
    scale = s;
  }
  
  void setField(FlowField f){
    for(Boid boid : boids){
      boid.setField(f);
    }
  }
  
  void add(){
    Boid boid = new Boid(random(0, width), random(0, height), scale);
    boids.add(boid);
  }
  
  void draw(){
    //println(boids.size());
    for (Boid boid : boids){
      ArrayList<Boid> nearest = sorted.get_nearest(boid, boid.too_far);
      boid.applyBehaviors(nearest);
      //boid.applyBehaviors(boids);
      boid.update();
      boid.draw();
   }
  }
}

class Boid{
  PVector position;
  PVector velocity;
  PVector acceleration;
  float size;
  float max_speed;
  float max_accel;
  float targ_radius;
  int wonder_lim;
  int wonder_step;
  float wonder_weight;
  float path_weight;
  PVector wonder_targ;
  float wonder_theta;
  float near;
  float align_near;
  float far;
  float too_far;
  color c;
  FlowField field;
  FloatList rel_dists;
  ArrayList<PVector> rel_positions;
  ArrayList<PVector> velocities;
  
  Boid(float x, float y, float s){
   position = new PVector(x, y);
   velocity = new PVector(0, -1);
   acceleration = new PVector(0, 0);
   wonder_targ = new PVector(0, 0);
   wonder_theta = 0;
   wonder_step = 0;
   wonder_lim = 20;
   size = s;
   max_speed = 5.0;
   max_accel = 0.15;
   targ_radius = 50;
   near = 30;
   far = 150;
   too_far = 150;
   align_near = 80;
   c = color(200, 100);
   rel_dists = new FloatList();
   rel_positions = new ArrayList<PVector>();
   velocities = new ArrayList<PVector>();
  }
  
  void draw(){
   fill(c);
   stroke(255);
   pushMatrix();
   beginShape(TRIANGLES);
   translate(position.x, position.y);
   rotate(velocity.heading()+PI/2);
   vertex(0, -size*2);
   vertex(-size, size*2);
   vertex(size, size*2);
   endShape();
   popMatrix();
  }
  
  void applyForce(PVector F){
    acceleration.add(F); 
  }
  
  private void compute_rel_positions(ArrayList<Boid> boids){
    rel_dists.clear();
    rel_positions.clear();
    velocities.clear();
    for (Boid boid : boids){
      PVector rel_pos = PVector.sub(position, boid.position);
      rel_positions.add(rel_pos);
      rel_dists.append(rel_pos.magSq());
      velocities.add(boid.velocity);
    }
  }
  
  void applyBehaviors(ArrayList<Boid> boids){
    PVector separate_f;
    PVector wonder_f;
    PVector align_f;
    PVector cohesion_f;
    PVector repel_f;
    PVector angleDrift_f;
    
    //PVector seek_f = findTarget(new PVector(mouseX, mouseY));
    wonder_f = wonder();
    repel_f = repel(new PVector(mouseX, mouseY), 100);
    
    if (!USE_BOIDS){
      compute_rel_positions(boids);
      separate_f = avoid();
      align_f = align();
      cohesion_f = cohesion();
      angleDrift_f = angleDrift(boids);
    }
    else{
      separate_f = avoid(boids);
      align_f = align(boids);
      cohesion_f = cohesion(boids);
      angleDrift_f = angleDrift(boids);
    }
    
    separate_f.mult(0.3);
    angleDrift_f.mult(5.0);
    wonder_f.mult(sin(random(0, PI))*0.01);
    align_f.mult(0.9);
    cohesion_f.mult(0.9);
    repel_f.mult(5);
    //seek_f.mult(0.9);
    applyForce(separate_f);
    applyForce(wonder_f);
    applyForce(align_f);
    applyForce(cohesion_f);
    applyForce(repel_f);
    position.add(angleDrift_f);
    //applyForce(drift_f);
    //applyForce(seek_f);
  }
  
  void setField(FlowField f){
    field = f;
  }
  
  PVector repel(PVector threat, float radius){
    PVector steer_f = new PVector(0, 0);
    if (PVector.sub(position, threat).magSq() < radius*radius){
      // In threat zone!
      PVector repel_dir = PVector.sub(position, threat);
      float mult = (repel_dir.magSq()-(radius*radius))-1.0;
      mult *= mult;
      steer_f = PVector.sub(repel_dir.mult(mult), velocity);
      steer_f.limit(max_accel);
    }
    
    return steer_f;
  }
  
  PVector angleDrift(ArrayList<Boid> boids){
    PVector steer_f = new PVector(0, 0);
    int size = 0;
    for(Boid boid : boids){
      //if(size > 10) continue;
      PVector rel_pos = PVector.sub(boid.position, position);
      if (rel_pos.magSq() == 0 || rel_pos.magSq() > near*near) continue;
      rel_pos.normalize();
      PVector norm_vel = new PVector(velocity.x, velocity.y);
      norm_vel.normalize();
      float dot = rel_pos.dot(norm_vel);
      if (dot > 0.70710677) continue;
      float side = norm_vel.x - rel_pos.x;
      PVector dir;
      if (norm_vel.x > rel_pos.x){
        dir = new PVector(velocity.y, velocity.x);
      }
      else{
        dir = new PVector(-velocity.y, velocity.x);
      }
      dir.mult( (dot-0.70710677)/(1.0-0.70710677)*4 );
      size++;
      steer_f.add(dir);
    }
    
    if (size == 0){
      return steer_f;
    }
    steer_f.div(size);
    steer_f.setMag(max_speed);
    //steer_f.sub(velocity);
    steer_f.limit(max_accel);
    return steer_f;
  }
  
  PVector cohesion(){
    PVector steer_f = new PVector(0, 0);
    int size = 0;
    for(int i=0; i < rel_positions.size(); i++){
      if (rel_dists.get(i) < far*far || rel_dists.get(i) > too_far*too_far || rel_dists.get(i) < near*near) continue;
      //rel_pos.normalize();
      size++;
      steer_f.add(PVector.mult(rel_positions.get(i), -1));
    }
    
    if (size == 0){
      return steer_f;
    }
    steer_f.div(size);
    steer_f.setMag(max_speed);
    steer_f.sub(velocity);
    steer_f.limit(max_accel);
    return steer_f;
  }
  
  PVector cohesion(ArrayList<Boid> boids){
    PVector steer_f = new PVector(0, 0);
    int size = 0;
    for(Boid boid : boids){
      PVector rel_pos = PVector.sub(boid.position, position);
      if (rel_pos.magSq() < far*far || rel_pos.magSq() > too_far*too_far || rel_pos.magSq() < near*near) continue;
      //rel_pos.normalize();
      size++;
      steer_f.add(rel_pos);
    }
    
    if (size == 0){
      return steer_f;
    }
    steer_f.div(size);
    steer_f.setMag(max_speed);
    steer_f.sub(velocity);
    steer_f.limit(max_accel);
    return steer_f;
  }
  
  PVector align(){
    PVector steer_f = new PVector(0, 0);
    int size = 0;
    for(int i=0; i < rel_positions.size(); i++){
      PVector rel_pos = PVector.mult(rel_positions.get(i), -1);
      Float rel_dist = rel_dists.get(i);
      if (rel_dist == 0 || rel_dist > near*near) continue;
      //rel_pos.normalize();
      PVector n_pos = new PVector(rel_pos.x, rel_pos.y);
      n_pos.normalize();
      PVector norm_vel = new PVector(velocity.x, velocity.y);
      norm_vel.normalize();
      if (n_pos.dot(norm_vel) < 0.70710677) continue;
      size++;
      steer_f.add(velocities.get(i));
    }
    
    if (size == 0){
      return steer_f;
    }
    steer_f.div(size);
    steer_f.setMag(max_speed);
    steer_f.sub(velocity);
    steer_f.limit(max_accel);
    return steer_f;
  }
  
  PVector align(ArrayList<Boid> boids){
    PVector steer_f = new PVector(0, 0);
    int size = 0;
    for(Boid boid : boids){
      PVector rel_pos = PVector.sub(boid.position, position);
      if (rel_pos.magSq() == 0 || rel_pos.magSq() > near*near) continue;
      //rel_pos.normalize();
      PVector n_pos = new PVector(rel_pos.x, rel_pos.y);
      n_pos.normalize();
      PVector norm_vel = new PVector(velocity.x, velocity.y);
      norm_vel.normalize();
      if (n_pos.dot(norm_vel) < 0.70710677) continue;
      size++;
      steer_f.add(boid.velocity);
    }
    
    if (size == 0){
      return steer_f;
    }
    steer_f.div(size);
    steer_f.setMag(max_speed);
    steer_f.sub(velocity);
    steer_f.limit(max_accel);
    return steer_f;
  }
  
  PVector avoid(){
    PVector steer_f = new PVector(0, 0);
    int size = 0;
    for(int i=0; i < rel_positions.size(); i++){
      PVector dir = rel_positions.get(i);
      Float dist = rel_dists.get(i);
      if(dist > (near*near) || dist <= 0) continue;
      if(dist < (near*near)){
        //dir.normalize();
        steer_f.add(dir);
        size++;
      }
    }
    if (steer_f.magSq() == 0) return steer_f;
    steer_f.div(size);
    steer_f.setMag(max_speed);
    steer_f.sub(velocity);
    steer_f.limit(max_accel);
    return steer_f;
  }
  
  PVector avoid(ArrayList<Boid> boids){
    PVector steer_f = new PVector(0, 0);
    int size = 0;
    for(Boid boid : boids){
      PVector dir = PVector.sub(position, boid.position);
      if(dir.magSq() > (near*near) || dir.magSq() <= 0) continue;
      if(dir.magSq() < (near*near)){
        //dir.normalize();
        steer_f.add(dir);
        size++;
      }
    }
    if (steer_f.magSq() == 0) return steer_f;
    steer_f.div(size);
    steer_f.setMag(max_speed);
    steer_f.sub(velocity);
    steer_f.limit(max_accel);
    return steer_f;
  }
  
  PVector findTarget(PVector targ_pos){
    PVector targ_dir = PVector.sub(targ_pos, position);
    float local_max = max_speed;
    if (targ_dir.magSq() < 30*30) local_max = (targ_dir.magSq()/(30*30));
    targ_dir.setMag(local_max);
    PVector steer_f = PVector.sub(targ_dir, velocity);
    steer_f.limit(max_accel);
    return steer_f;
  }
  
  PVector wonder(){
    if(field != null){
      return fieldLookup(field);
    }
    else{
      return wonderSelf();
    }
  }
  
  PVector wonderSelf(){
    wonder_step += 1;
    if(wonder_step >= wonder_lim) wonder_step = 0;
    if(wonder_step == 0) wonder_theta = random(-180, 180);
    if(wonder_step % 3 == 0) wonder_theta += random(-1, 1);
    PVector pos_prime = PVector.add(position, PVector.mult(velocity, targ_radius*2));
    wonder_targ = new PVector(cos(wonder_theta)*velocity.x - sin(wonder_theta)*velocity.y, sin(wonder_theta)*velocity.x + cos(wonder_theta)*velocity.y);
    wonder_targ.mult(targ_radius);
    wonder_targ.add(pos_prime);
    return findTarget(wonder_targ);
  }
  
  PVector fieldLookup(FlowField field){
    PVector steer_f = field.lookup(position);
    steer_f.setMag(max_speed);
    steer_f.sub(velocity);
    steer_f.limit(max_accel);
    return steer_f; 
  }
  
  PVector findPath(Path path){
    ArrayList<PVector> norm_positions = new ArrayList<PVector>();
    PVector pos_prime = PVector.add(position, PVector.mult(velocity, path.radius*1.5));
    for (int i = 0; i < path.points.size()-1; i++){
      PVector start = path.points.get(i);
      PVector end = path.points.get(i+1);
      PVector line_dir = PVector.sub(end, start).normalize();
      PVector dir = PVector.sub(pos_prime, start);
      if (dir.dot(line_dir) < 0) continue;
      PVector norm_pos = PVector.mult(line_dir, dir.dot(line_dir));
      if (norm_pos.magSq() > PVector.sub(end, start).magSq()) continue;
      norm_pos.add(start);
      norm_positions.add(norm_pos);
    }
    
    int choice = 0;
    if (norm_positions.size() == 0) {
      return new PVector(0, 0);
    }
    float last_dist = PVector.dist(pos_prime, norm_positions.get(0));
    
    for (int i = 1; i < norm_positions.size(); i++){
      float dist = PVector.dist(pos_prime, norm_positions.get(i));
      if (dist < last_dist){
        last_dist = dist;
        choice = i;
      }
    }
    return findTarget(norm_positions.get(choice));
  }
  
  void update(){
    velocity.add(acceleration);
    velocity.limit(max_speed);
    position.add(velocity);
    if (position.y < -size) position.y = height + size;
    if (position.x < -size) position.x = width + size;
    if (position.y > height + size) position.y = -size;
    if (position.x > width + size) position.x = -size;
    acceleration.mult(0);
  }
}
