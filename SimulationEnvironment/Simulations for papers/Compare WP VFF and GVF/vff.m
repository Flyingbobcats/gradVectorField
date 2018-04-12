classdef vff

    
    properties
    goalPosX = []
    goalPosY = []
    
    obstPosX = []
    obstPosY = []
    
    Fcr = -2
    Fct = 0.8
    
    detectionRadius = 7.5
    width = 5
    n = 2
    
    c = 0




    fx = [];
    fy = []; 
    cmd_heading = []
        
    end
    
    methods
        
        function self = heading(self,x,y)   
        
        d = sqrt((x-self.obstPosX)^2+(y-self.obstPosY)^2);
        dg = sqrt((x-self.goalPosX)^2+(y-self.goalPosY)^2);
        %Assume perfect detection of obstacle within a certain radius
        if d<=self.detectionRadius
            self.c = self.c+100;
        else
            self.c = 0;
        end

        FrepX = self.Fcr*self.width*self.c / d^self.n *(self.obstPosX-x)/d;
        FrepY = self.Fcr*self.width*self.c / d^self.n *(self.obstPosY-y)/d;

        Fgoalx = self.Fct*(self.goalPosX-x)/dg;
        Fgoaly = self.Fct*(self.goalPosY-y)/dg;

        Fx = FrepX + Fgoalx;
        Fy = FrepY + Fgoaly;
        
        self.cmd_heading = atan2(Fy,Fx);


        end
        
        function self = setup(self,goal,obst)
            self.goalPosX = goal(1);
            self.goalPosY = goal(2);
            
            self.obstPosX = obst(1);
            self.obstPosY = obst(2);
            
        end
    
    end
end

