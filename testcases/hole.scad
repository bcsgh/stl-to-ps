$fn=64;
difference() {
  scale([5,5,1]) cube();
  translate([2,3,-1]) scale([1,1,3]) cylinder(r=1);
  translate([1,1,-1]) scale([1,1,3]) cylinder(r=.5); 
  translate([-.5,-.5,-1]) {
    scale([2,1.5,3]) cube();
    scale([1.5,2,3]) cube();
  }
}