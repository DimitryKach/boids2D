import java.util.Comparator;
import java.util.Collections;

public static class KDTree{
  ArrayList<Boid> input_list;
  Node root;
  ArrayList<Node> nodes;
  
  KDTree(ArrayList<Boid> l){
    input_list = l;
    root = new Node(0);
    nodes = new ArrayList<Node>();
    nodes.add(root);
    generate(root, input_list);
  }
  private final static Comparator<Boid> SORTX = new SortByX();
  private final static Comparator<Boid> SORTY = new SortByY();
  
  public void regenerate(){
    nodes.clear();
    root = new Node(0);
    nodes.add(root);
    generate(root, input_list);
  }
  
  private void generate(Node node, ArrayList<Boid> boids){
    if ((node.depth&1) == 0){
      Collections.sort(boids, SORTX);
    }
    if ((node.depth&1) == 1){
      Collections.sort(boids, SORTY);
    }
    int mid = boids.size() >> 1;
    node.boid = boids.get(mid);
    if (boids.size() > 1){
      ArrayList<Boid> l = new ArrayList<Boid>();
      
      for (int i = 0; i < mid; i++){
        l.add(boids.get(i));
      }
      node.L = new Node(node.depth + 1);
      nodes.add(node.L);
      generate(node.L, l);
      
      if (boids.size() > 2){
        ArrayList<Boid> r = new ArrayList<Boid>();
        for (int i = mid+1; i < boids.size(); i++){
          r.add(boids.get(i));
        }
        node.R = new Node(node.depth + 1);
        nodes.add(node.R);
        generate(node.R, r);
      }
    }
  }
  
  public ArrayList<Boid> get_nearest(Boid boid, float dist){
    NearestNs nearest = new NearestNs(boid, dist);
    search_tree(nearest, root);
    ArrayList<Boid> nn_boids = new ArrayList<Boid>();
    for (Node n : nearest.neighbors){
      nn_boids.add(n.boid);
    }
    return nn_boids;
  }
  
  private void search_tree(NearestNs nearest, Node node){
    if (!nearest.full && nearest.near+nearest.mid+nearest.far < nearest.max*2){
      if(node.is_leaf()){
        float n_dist = PVector.sub(nearest.position, node.boid.position).magSq();
        if ( n_dist < nearest.dist*nearest.dist ) nearest.add(node, n_dist);
      }
      else{
        float n_dist = PVector.sub(nearest.position, node.boid.position).magSq();
        if ( n_dist < nearest.dist*nearest.dist ){
          nearest.add(node, n_dist);
          if (node.L != null) search_tree(nearest, node.L);
          if (node.R != null) search_tree(nearest, node.R);
        }
        else{
          float diff = (node.depth&1) == 0 ? nearest.position.x - node.boid.position.x : nearest.position.y - node.boid.position.y;
          Node to_search = (diff < 0) ? node.L : node.R;
          if (to_search != null) search_tree( nearest, to_search);
        }
      }
    }
  }
  
  public static class Node{
    int depth;
    Boid boid;
    Node L, R;
    
    public Node(int depth){
      this.depth = depth;
    }
    
    boolean is_leaf(){
      return (L == null) | (R == null);
    }
    
  }
  
  public static class NearestNs{
    float dist;
    ArrayList<Node> neighbors;
    PVector position;
    float d_mid, d_far;
    int near, mid, far;
    int max;
    float d_mid_shrink;
    boolean full;
    
    public NearestNs(Boid boid, float d){
      dist = d;
      position = boid.position;
      neighbors = new ArrayList<Node>();
      max = 500;
      d_mid = (dist*dist)/3;
      d_far = 2*(dist*dist)/3;
      near = mid = far = 0;
      full = false;
    }
    
    public void add(Node node, float n_dist){
      if (n_dist < d_mid){
        if (near < max){
          near++;
          neighbors.add(node);
        }
        else{
          full = true;
        }
      }
      if (n_dist >= d_mid && n_dist < d_far){
        if (mid < max/2){
          mid++;
          neighbors.add(node);
        }
      }
      if (n_dist >= d_far){
        if (far < max/2){
          far++;
          neighbors.add(node);
        }
      }
    }
  }
}

public static final class SortByX implements Comparator<Boid>{
  
  public int compare(Boid boid1, Boid boid2){
    if (boid1.position.x < boid2.position.x){
      return -1;
    }
    else if (boid1.position.x > boid2.position.x){
      return 1;
    }
    else{
      return 0;
    }
  }
}

public static final class SortByY implements Comparator<Boid>{
  
 public int compare(Boid boid1, Boid boid2){
    if (boid1.position.y < boid2.position.y){
      return -1;
    }
    else if (boid1.position.y > boid2.position.y){
      return 1;
    }
    else{
      return 0;
    }
  }
}
