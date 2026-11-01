

struct Planar3r{
    up1 : f64,
    up2 : f64,
    up3 : f64,
}

struct StateJoint{
    rz : f64,
    planar3r : Planar3r,
    gr : f64,
    gc : f64,
    l1 : f64,
    l2 : f64,
    l3 : f64,
}

struct EndEffectorPos{
    x : f64,
    y : f64,
    t : f64,
}

impl StateJoint{

    fn new(l1: f64, l2: f64, l3: f64) -> StateJoint {
        StateJoint {rz:0.0, planar3r:Planar3r{up1:0.0,up2:0.0,up3:0.0},gr:0.0,gc:0.0, l1: l1, l2: l2 , l3: l3}
    }

    fn get_pose_3r(&self) -> EndEffectorPos {
        EndEffectorPos {
            x : self.l1*self.planar3r.up1.cos()+
                self.l2*(self.planar3r.up1+self.planar3r.up2).cos()+
                self.l3*(self.planar3r.up1+self.planar3r.up2+self.planar3r.up3).cos(),
            y : self.l1*self.planar3r.up1.sin()+
                self.l2*(self.planar3r.up1+self.planar3r.up2).sin()+
                self.l3*(self.planar3r.up1+self.planar3r.up2+self.planar3r.up3).sin(),
            t : self.planar3r.up1+self.planar3r.up2+self.planar3r.up3,
        }
    }

    fn solve_ik_3r(&self, end_effector_pos:EndEffectorPos) -> Planar3r{
        println!("end effector pose ({}, {}, {})", end_effector_pos.x, end_effector_pos.y, end_effector_pos.t);
        Planar3r {
            up1 : 0.0,
            up2 : 0.0,
            up3 : 0.0,
        }
    }

    fn set_new_pose_planar(&mut self, planar3r : Planar3r){
        self.planar3r = planar3r
    }

}

fn main() {
    let mut sol = StateJoint::new(0.2,0.3,1.0);
    let pos = sol.get_pose_3r();
    println!("end effector pose ({}, {}, {})", pos.x, pos.y, pos.t);
    sol.set_new_pose_planar(Planar3r{up1:std::f64::consts::PI/2.0,up2:0.0,up3:0.0});
    let pos = sol.get_pose_3r();
    println!("end effector pose ({}, {}, {})", pos.x, pos.y, pos.t);


}