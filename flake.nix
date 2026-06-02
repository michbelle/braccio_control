{
  inputs = {
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/master";
    nixpkgs.follows = "nix-ros-overlay/nixpkgs";  # IMPORTANT!!!
    nixgl.url = "github:nix-community/nixGL";
  };
  outputs = { self, nix-ros-overlay, nixpkgs, nixgl }:
    nix-ros-overlay.inputs.flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs {
          inherit system;
          # overlays = [ nix-ros-overlay.overlays.default nixgl.overlay ];
          overlays = [ nix-ros-overlay.overlays.default ];
        };
      in {
        devShells.default = pkgs.mkShell {
          name = "Example project";
          packages = [
            # pkgs.nixgl.auto.nixGLDefault
            pkgs.colcon
            # ... other non-ROS packages
            (with pkgs.rosPackages.lyrical; buildEnv {
              paths = [
                desktop-full
                # ... other ROS packages
              ];
            })
          ];
        };
      });
  nixConfig = {
    extra-substituters = [ "https://ros.cachix.org" ];
    extra-trusted-public-keys = [ "ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo=" ];
  };
}