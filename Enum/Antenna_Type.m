 classdef Antenna_Type < int32
     enumeration
         H              (0)
         Array          (1)
         Isotropic      (2)
         NoBackH        (3)     % same as H, but remove back lobe (Unrealistic, for testing only)
     end
 end