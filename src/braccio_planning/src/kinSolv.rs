

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
        let mut sol: Planar3r =  Planar3r {
                up1 : 0.0,
                up2 : 0.0,
                up3 : 0.0,
            };
        
        let x_ = end_effector_pos.x - self.l3*end_effector_pos.t.cos();
        let y_ = end_effector_pos.y - self.l3*end_effector_pos.t.sin();

        let c_a2: f64 = (x_.powf(2.0) + y_.powf(2.0)-self.l1.powf(2.0)-self.l2.powf(2.0))/(2.0*self.l1*self.l2);
        let mut s_a2: f64 = (1.0-c_a2.powf(2.0)).sqrt();

        if self.planar3r.up2 < 0.0 {
            s_a2 = -s_a2;
        }

        sol.up2 = f64::atan2(s_a2, c_a2);
        sol.up1 = f64::atan2(y_, x_)-f64::atan2(self.l2*sol.up2.sin(),self.l1+self.l2*sol.up2.cos());
        sol.up3 = end_effector_pos.t - sol.up1 -sol.up2;

        sol

    }

    fn set_new_pose_planar(&mut self, planar3r : &Planar3r){
        self.planar3r.up1 = planar3r.up1;
        self.planar3r.up2 = planar3r.up2;
        self.planar3r.up3 = planar3r.up3;
    }

}

#[cfg(test)]
mod tests {

    use super::*;

    #[test]
    fn direct_kin(){
        let mut sol = StateJoint::new(0.2,0.3,1.0);
        let pos = sol.get_pose_3r();
        println!("end effector pose ({}, {}, {})", pos.x, pos.y, pos.t);
        assert!(
            pos.x == 0.2 + 0.3 + 1.0 && pos.y == 0.0 && pos.t ==  0.0
        );
        let pos1 = Planar3r{up1: std::f64::consts::PI/2.0,up2:0.0,up3:0.0};
        sol.set_new_pose_planar(&pos1);
        let pos2 = sol.get_pose_3r();
        println!("end effector pose ({}, {}, {})", pos2.x, pos2.y, pos2.t);
        assert!(pos2.x < 0.001);
        assert!(pos2.y == 1.5);
        assert!(pos2.t ==  std::f64::consts::PI/2.0);
    }

    #[test]
    fn ik(){
        let mut sol = StateJoint::new(0.2,0.3,1.0);
        let angles = Planar3r{up1: 0.7,up2:0.2,up3:-0.1};
        sol.set_new_pose_planar(&angles);
        let pos = sol.get_pose_3r();
        let end = sol.solve_ik_3r(pos);
        println!("end effector pose ({}, {}, {})", end.up1,end.up2,end.up3);
        println!("end effector pose ({}, {}, {})", angles.up1,angles.up2,angles.up3);
        assert!((end.up1-angles.up1).abs()<0.001);
        assert!((end.up2-angles.up2).abs()<0.001);
        assert!((end.up3-angles.up3).abs()<0.001);
    }


}