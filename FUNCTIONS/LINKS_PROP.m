function LINKS = LINKS_PROP()


%% LEG 1 --------------------------------------------------------

%% LEG 1: PROXIMAL (Kg,mm)

    Prox1 = struct;
    Prox1.m = 8.38027712158635e-003;  

    % Center of mass vector: Taken at the body frame coordinate system.
    Prox1.R_Bf = [0;15.00104516;-30.52515296];

    % Moments of inertia: Taken at the center of mass and aligned
    % with the body frame coordinate system.    
    Prox1_Ixx_CM = 1.61482064;
    Prox1_Iyy_CM = 1.02864800;
    Prox1_Izz_CM = 0.71808841;
    Prox1_Ixy_CM = 0;
    Prox1_Ixz_CM = 0;
    Prox1_Iyz_CM = -0.68252713;
    
    Prox1.I_CM = [Prox1_Ixx_CM , Prox1_Ixy_CM , Prox1_Ixz_CM;
                  Prox1_Ixy_CM , Prox1_Iyy_CM , Prox1_Iyz_CM;
                  Prox1_Ixz_CM , Prox1_Iyz_CM , Prox1_Izz_CM];

    % Moments of inertia: Taken at the body frame coordinate system.    
    Prox1_Ixx_Bf = 11.30926362;
    Prox1_Iyy_Bf = 8.83726554;
    Prox1_Izz_Bf = 2.60391385;
    Prox1_Ixy_Bf = 0.00000002;
    Prox1_Ixz_Bf =  -0.00000003;
    Prox1_Iyz_Bf = 3.15487950;
    
    Prox1.I_Bf = [Prox1_Ixx_Bf , Prox1_Ixy_Bf , Prox1_Ixz_Bf;
                  Prox1_Ixy_Bf , Prox1_Iyy_Bf , Prox1_Iyz_Bf;
                  Prox1_Ixz_Bf , Prox1_Iyz_Bf , Prox1_Izz_Bf];

%% LEG 1: DISTAL (Kg,mm)

    Dist1 = struct;
    Dist1.m = 6.71322338197224e-003;  
    
    % Center of mass vector: Taken at the body frame coordinate system.
    Dist1.R_Bf = [10.98040456;19.39131205;0.00000884];

    % Moments of inertia: Taken at the center of mass and aligned
    % with the body frame coordinate system.     
    Dist1_Ixx_CM = 0.23122138;
    Dist1_Iyy_CM = 0.64457646;
    Dist1_Izz_CM = 0.84349883;
    Dist1_Ixy_CM = 0.29369756;
    Dist1_Ixz_CM = 0.00000008;
    Dist1_Iyz_CM = 0.00000053;
    
    Dist1.I_CM = [Dist1_Ixx_CM , Dist1_Ixy_CM , Dist1_Ixz_CM;
                  Dist1_Ixy_CM , Dist1_Iyy_CM , Dist1_Iyz_CM;
                  Dist1_Ixz_CM , Dist1_Iyz_CM , Dist1_Izz_CM];    
        

    % Moments of inertia: Taken at the body frame coordinate system.
    Dist1_Ixx_Bf = 2.75621057;
    Dist1_Iyy_Bf = 1.45419756;
    Dist1_Izz_Bf = 4.17810911;
    Dist1_Ixy_Bf = -1.13608721;
    Dist1_Ixz_Bf = -0.00000057;
    Dist1_Iyz_Bf = -0.00000062;
    
    Dist1.I_Bf = [Dist1_Ixx_Bf , Dist1_Ixy_Bf , Dist1_Ixz_Bf;
                  Dist1_Ixy_Bf , Dist1_Iyy_Bf , Dist1_Iyz_Bf;
                  Dist1_Ixz_Bf , Dist1_Iyz_Bf , Dist1_Izz_Bf];    

%% END LEG 1 ----------------------------------------------------    
    

%% LEG 2 --------------------------------------------------------

%% LEG 2: PROXIMAL (Kg,mm)

    Prox2 = struct;
    Prox2.m = 9.02521759129636e-003;  
    
    % Center of mass vector: Taken at the body frame coordinate system.
    Prox2.R_Bf = [0;13.67861771;-28.93291259];

    % Moments of inertia: Taken at the center of mass and aligned
    % with the body frame coordinate system. 
    Prox2_Ixx_CM = 1.52004672;
    Prox2_Iyy_CM = 0.78628394;
    Prox2_Izz_CM = 0.88151706;
    Prox2_Ixy_CM = 0;
    Prox2_Ixz_CM = 0;
    Prox2_Iyz_CM = -0.62094758;
    
    Prox2.I_CM = [Prox2_Ixx_CM , Prox2_Ixy_CM , Prox2_Ixz_CM;
                  Prox2_Ixy_CM , Prox2_Iyy_CM , Prox2_Iyz_CM;
                  Prox2_Ixz_CM , Prox2_Iyz_CM , Prox2_Izz_CM];    

    % Moments of inertia: Taken at the body frame coordinate system.
    Prox2_Ixx_Bf = 10.76383720;
    Prox2_Iyy_Bf = 8.34141484;
    Prox2_Izz_Bf = 2.57017664;
    Prox2_Ixy_Bf = 0.00000002;
    Prox2_Ixz_Bf = -0.00000004;
    Prox2_Iyz_Bf = 2.95089286;
    
    Prox2.I_Bf = [Prox2_Ixx_Bf , Prox2_Ixy_Bf , Prox2_Ixz_Bf;
                  Prox2_Ixy_Bf , Prox2_Iyy_Bf , Prox2_Iyz_Bf;
                  Prox2_Ixz_Bf , Prox2_Iyz_Bf , Prox2_Izz_Bf];    
    
    
%% LEG 2: DISTAL (Kg,mm)   
  
    Dist2 = struct;
    Dist2.m = 6.71327365781553e-003;  
    
    % Center of mass vector: Taken at the body frame coordinate system.
    Dist2.R_Bf = [10.98040499;19.39131230;0.00000884];

    % Moments of inertia: Taken at the center of mass and aligned
    % with the body frame coordinate system.     
    Dist2_Ixx_CM = 0.23122138;
    Dist2_Iyy_CM = 0.64457646;
    Dist2_Izz_CM = 0.84349883;
    Dist2_Ixy_CM = 0.29369756;
    Dist2_Ixz_CM = 0.00000008;
    Dist2_Iyz_CM = 0.00000053;
    
    Dist2.I_CM = [Dist2_Ixx_CM , Dist2_Ixy_CM , Dist2_Ixz_CM;
                  Dist2_Ixy_CM , Dist2_Iyy_CM , Dist2_Iyz_CM;
                  Dist2_Ixz_CM , Dist2_Iyz_CM , Dist2_Izz_CM];         

    % Moments of inertia: Taken at the body frame coordinate system.
    Dist2_Ixx_Bf = 2.75621063;
    Dist2_Iyy_Bf = 1.45419762;
    Dist2_Izz_Bf = 4.17810924;
    Dist2_Ixy_Bf = -1.13608729;
    Dist2_Ixz_Bf = -0.00000057;
    Dist2_Iyz_Bf = -0.00000062;   
    
    Dist2.I_Bf = [Dist2_Ixx_Bf , Dist2_Ixy_Bf , Dist2_Ixz_Bf;
                  Dist2_Ixy_Bf , Dist2_Iyy_Bf , Dist2_Iyz_Bf;
                  Dist2_Ixz_Bf , Dist2_Iyz_Bf , Dist2_Izz_Bf];      
    
%% END LEG 2 ----------------------------------------------------      
 

%% LEG 3 --------------------------------------------------------

%% LEG 3: PROXIMAL (Kg,mm)

    Prox3 = struct;
    Prox3.m = 9.35728764600804e-003;  
    
    % Center of mass vector: Taken at the body frame coordinate system.
    Prox3.R_Bf = [0;12.86419686;-26.50463644];

    % Moments of inertia: Taken at the center of mass and aligned
    % with the body frame coordinate system. 
    Prox3_Ixx_CM = 1.35287838;
    Prox3_Iyy_CM = 0.54378817;
    Prox3_Izz_CM = 0.96364276;
    Prox3_Ixy_CM = 0.00000002;
    Prox3_Ixz_CM = -0.00000001;
    Prox3_Iyz_CM = -0.49660916;
    
    Prox3.I_CM = [Prox3_Ixx_CM , Prox3_Ixy_CM , Prox3_Ixz_CM;
                  Prox3_Ixy_CM , Prox3_Iyy_CM , Prox3_Iyz_CM;
                  Prox3_Ixz_CM , Prox3_Iyz_CM , Prox3_Izz_CM];      

    % Moments of inertia: Taken at the body frame coordinate system.
    Prox3_Ixx_Bf = 9.47484813;
    Prox3_Iyy_Bf = 7.11724318;
    Prox3_Izz_Bf = 2.51215751;
    Prox3_Ixy_Bf = 0.00000001;
    Prox3_Ixz_Bf = 0;
    Prox3_Iyz_Bf = 2.69385977;
    
    Prox3.I_Bf = [Prox3_Ixx_Bf , Prox3_Ixy_Bf , Prox3_Ixz_Bf;
                  Prox3_Ixy_Bf , Prox3_Iyy_Bf , Prox3_Iyz_Bf;
                  Prox3_Ixz_Bf , Prox3_Iyz_Bf , Prox3_Izz_Bf];       
    
%% LEG 3: DISTAL (Kg,mm)      

    Dist3 = struct;
    Dist3.m = 6.71323366453039e-003;  
    
    % Center of mass vector: Taken at the body frame coordinate system.
    Dist3.R_Bf = [10.98040449;19.39131205;0.00000862];

    % Moments of inertia: Taken at the center of mass and aligned
    % with the body frame coordinate system.     
    Dist3_Ixx_CM = 0.23122138;
    Dist3_Iyy_CM = 0.64457646;
    Dist3_Izz_CM = 0.84349883;
    Dist3_Ixy_CM = 0.29369756;
    Dist3_Ixz_CM = 0.00000008;
    Dist3_Iyz_CM = 0.00000053;
    
    Dist3.I_CM = [Dist3_Ixx_CM , Dist3_Ixy_CM , Dist3_Ixz_CM;
                  Dist3_Ixy_CM , Dist3_Iyy_CM , Dist3_Iyz_CM;
                  Dist3_Ixz_CM , Dist3_Iyz_CM , Dist3_Izz_CM];      

    % Moments of inertia: Taken at the body frame coordinate system.
    Dist3_Ixx_Bf = 2.75621057;
    Dist3_Iyy_Bf = 1.45419755;
    Dist3_Izz_Bf = 4.17810910;
    Dist3_Ixy_Bf = -1.13608720;
    Dist3_Ixz_Bf = -0.00000056;
    Dist3_Iyz_Bf = -0.00000059;  
    
    Dist3.I_Bf = [Dist3_Ixx_Bf , Dist3_Ixy_Bf , Dist3_Ixz_Bf;
                  Dist3_Ixy_Bf , Dist3_Iyy_Bf , Dist3_Iyz_Bf;
                  Dist3_Ixz_Bf , Dist3_Iyz_Bf , Dist3_Izz_Bf];      
    
%% END LEG 3 ----------------------------------------------------  

%% PLATFORM -----------------------------------------------------


    Platform = struct;
    Platform.m = 4.38998768456049e-002;  
    
    % Center of mass vector: Taken at the body frame coordinate system.
    Platform.R_Bf = [0.00002029;-0.00003363;-8.06965072];

    % Moments of inertia: Taken at the center of mass and aligned
    % with the body frame coordinate system.     
    Platform_Ixx_CM = 4.60332146;
    Platform_Iyy_CM = 4.60323587;
    Platform_Izz_CM = 7.91875023;
    Platform_Ixy_CM = -0.00001123;
    Platform_Ixz_CM = -0.00001721;
    Platform_Iyz_CM = 0.00000074;
    
    Platform.I_CM = [Platform_Ixx_CM , Platform_Ixy_CM , Platform_Ixz_CM;
                     Platform_Ixy_CM , Platform_Iyy_CM , Platform_Iyz_CM;
                     Platform_Ixz_CM , Platform_Iyz_CM , Platform_Izz_CM];  

    % Moments of inertia: Taken at the body frame coordinate system.
    Platform_Ixx_Bf = 7.46204091;
    Platform_Iyy_Bf = 7.46195532;
    Platform_Izz_Bf = 7.91875023;
    Platform_Ixy_Bf = -0.00001123;
    Platform_Ixz_Bf = -0.00001002;
    Platform_Iyz_Bf = -0.00001117;  
    
    Platform.I_Bf = [Platform_Ixx_Bf , Platform_Ixy_Bf , Platform_Ixz_Bf;
                     Platform_Ixy_Bf , Platform_Iyy_Bf , Platform_Iyz_Bf;
                     Platform_Ixz_Bf , Platform_Iyz_Bf , Platform_Izz_Bf];          
    
    

%% END PLATFORM -------------------------------------------------

%% OUTPUT -------------------------------------------------


LINKS = struct;
LINKS.PROXIMAL = {Prox1,Prox2,Prox3};
LINKS.DISTAL = {Dist1,Dist2,Dist3};
LINKS.PLATFORM = Platform;





end

