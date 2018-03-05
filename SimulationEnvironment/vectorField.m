%Main vector field object to store, create, and modify multiple vector
%fields

classdef vectorField
    %Master vector field class that handles all attractive fields and major
    %calling functions
    
    % =============== Functions inside of class instance =============
    % navf      - New attractive vector field
    % nrvf      - New repulsive vector field
    % xydomain  - Primarily for plotting purposes, updates xy space
    % sumFields - Sums together all attractive and repulsive vector fields
    % =====================================================================
    
    properties
        NormSummedFields     = true
        normAttractiveFields = true
        normRepulsiveFields  = false
        
        avfWeight = 1;
        rvfWeight = 1;
        avf = {};
        rvf = {};
        
        %Global properties
        xspace = linspace(-10,10,25);
        yspace = linspace(-10,10,25);
        n = 25;
    end
    
    methods
        %Add a vector field
        function self = navf(self,type)
            if strcmp(type,'circ')==1
                self.avf{length(self.avf)+1} = cndr;
            end
            
            if strcmp(type,'line')
                self.avf{length(self.avf)+1} = gvfLine;
            end
            %Give new field default settings
            self.avf{end}.xspace = self.xspace;
            self.avf{end}.yspace = self.yspace;
        end
        
        function self = nrvf(self,type)
            if strcmp(type,'circ')==1
                self.rvf{length(self.rvf)+1} = cndr;
                self.rvf{end}.G = -1;
                self.rvf{end}.H = 0;
            end
            
            if strcmp(type,'line')
                self.rvf{length(self.rvf)+1} = gvfLine;
            end
            %Give new field default settings
            self.rvf{end}.xspace = self.xspace;
            self.rvf{end}.yspace = self.yspace;
        end
        
        
        %Change the x and y doman
        function self = xydomain(self,s,x_c,y_c,n_new)
            x_space_new = linspace(-s+x_c,s+x_c,n_new);
            y_space_new = linspace(-s+y_c,s+y_c,n_new);
            self.xspace = x_space_new;
            self.yspace = y_space_new;
            self.n = n_new;
            
            for i = 1:length(self.avf)
                self.avf{i}.xspace = x_space_new;
                self.avf{i}.yspace = y_space_new;
                self.avf{i}.n = n_new;
            end
            
            for i = 1:length(self.rvf)
                self.rvf{i}.xspace = x_space_new;
                self.rvf{i}.yspace = y_space_new;
                self.rvf{i}.n = n_new;
            end
            
        end
        
        
        
        
        function [X,Y,Ut,Vt] = sumFields(self)
            
            if length(self.avf) + length(self.rvf) < 1
                warning('No vector fields to sum');
                X = NaN;
                Y = NaN;
                Ut = NaN;
                Vt = NaN;
                return
            end
            
            %Get all of the attractive fields
            Usa = cell(length(self.avf));
            Vsa = cell(length(self.avf));
            for i = 1:length(self.avf)
                if self.avf{i}.active == true
                    [X,Y,U,V] = self.avf{i}.ff;
                    Usa{i} = U;
                    Vsa{i} = V;
                end
            end
            
            %Summ all attractive fields into one set
            Uta = zeros(length(self.xspace));
            Vta = zeros(length(self.yspace));
            for i = 1:length(Usa)
                for j = 1:length(Usa{1})
                    for k = 1:length(Usa{1})
                        
                        Uta(j,k) = Uta(j,k)+Usa{i}(j,k);
                        Vta(j,k) = Vta(j,k)+Vsa{i}(j,k);
                        
                        if self.normAttractiveFields
                            N = norm([Uta(j,k),Vta(j,k)]);
                            Uta(j,k) = Uta(j,k)/N;
                            Vta(j,k) = Vta(j,k)/N;
                        end
                        
                    end
                end
            end
            
            
            
            
            %Geta all of the repulsive vector vields
            Usr = cell(length(self.rvf));
            Vsr = cell(length(self.rvf));
            for i = 1:length(self.rvf)
                if self.rvf{i}.active == true
                    [X,Y,U,V] = self.rvf{i}.ff;
                    Usr{i} = U;
                    Vsr{i} = V;
                end
            end
            
            
            %Sum together all of the repulsive vector fields
            Utr = zeros(length(self.xspace));
            Vtr = zeros(length(self.yspace));
            for i = 1:length(Usr)
                for j = 1:length(Usr{1})
                    for k = 1:length(Usr{1})
                        Utr(j,k) = Utr(j,k)+Usr{i}(j,k);
                        Vtr(j,k) = Vtr(j,k)+Vsr{i}(j,k);
                        
                        if self.normRepulsiveFields
                            N = norm([Utr(j,k),Vtr(j,k)]);
                            Utr(j,k) = Utr(j,k)/N;
                            Vtr(j,k) = Vtr(j,k)/N;
                        end
                    end
                end
            end
            
            
            
            %Sum together attractive and repulsive vector field
            Ut = NaN(length(self.xspace));
            Vt = NaN(length(self.xspace));
            for i = 1:length(self.xspace)
                for j = 1:length(self.yspace)
                    
                    Ut(i,j) = self.avfWeight*Uta(i,j)+self.rvfWeight*Utr(i,j);
                    Vt(i,j) = self.avfWeight*Vta(i,j)+self.rvfWeight*Vtr(i,j);
                    
                    if self.NormSummedFields
                        N = norm([Ut(i,j),Vt(i,j)]);
                        Ut(i,j) = Ut(i,j)/N;
                        Vt(i,j) = Vt(i,j)/N;
                    end
                end
            end
        end
        
        
        
       
        
        function [Ut,Vt] = heading(self,x,y)
            
            posx = x;
            posy = y;
            
            Usa = cell(length(self.avf));
            Vsa = cell(length(self.avf));
            for i = 1:length(self.avf)
                if self.avf{i}.active == true
                    [U,V] = self.avf{i}.comp(posx,posy);
                    Usa{i} = U;
                    Vsa{i} = V;
                end
            end
            
            %Repulsive Vector Fields
            Usr = cell(length(self.rvf));
            Vsr = cell(length(self.rvf));
            for i = 1:length(self.rvf)
                if self.rvf{i}.active == true
                    [U,V] = self.rvf{i}.comp(posx,posy);
                    Usr{i} = U;
                    Vsr{i} = V;
                end
            end
            
            Uta = zeros(1);
            Vta = zeros(1);
            for i = 1:length(Usa)
                for j = 1:length(Usa{1})
                    for k = 1:length(Usa{1})
                        Uta(j,k) = Uta(j,k)+Usa{i}(j,k);
                        Vta(j,k) = Vta(j,k)+Vsa{i}(j,k);
                    end
                end
            end
            
            Utr = zeros(1);
            Vtr = zeros(1);
            for i = 1:length(Usr)
                for j = 1:length(Usr{1})
                    for k = 1:length(Usr{1})
                        Utr(j,k) = Utr(j,k)+Usr{i}(j,k);
                        Vtr(j,k) = Vtr(j,k)+Vsr{i}(j,k);
                    end
                end
            end
            
            for i = 1:length(1)
                for j = 1:length(1)
                    Ut(i,j) = self.avfWeight*Uta(i,j)+self.rvfWeight*Utr(i,j);
                    Vt(i,j) = self.avfWeight*Vta(i,j)+self.rvfWeight*Vtr(i,j);
                end
            end
        end
        
        
        %=================== Plotting ====================================%
        function fig = pltff(self)
            [x,y,u,v] = self.sumFields;   
%             if sum(isnan(u(:))) > 0 || sum(isnan(v(:))) > 0
%                warning('sumFields returned NaN'); 
%             end
            
            quiver(x,y,u,v,'linewidth',1);
            axis equal
            
            fig = gca;
        end
        
        function pltPaths(self)
            for i = 1:length(self.avf)
                self.avf{i}.pltfnc;
            end
        end
        
        
        function pltDecay(self)
            for i = 1:length(self.rvf)
               self.rvf{i}.pltDecay; 
            end
        end
        %%%% ============= Helper / Debugger Functions ======================
        %Number of vector fields
        function numvf(self)
            length(self.avf)
        end
        
        
        
    end
    
end

