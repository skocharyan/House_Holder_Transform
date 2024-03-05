classdef (StrictDefaults) CustomQRMode < matlab.System
    % untitled Add summary here
    %
    % NOTE: When renaming the class name untitled, the file name
    % and constructor name must be updated to use the class name.
    %
    % This template includes most, but not all, possible properties, attributes,
    % and methods that you can implement for a System object in Simulink.

    % Public, tunable properties
    properties
      
    end

    % Public, non-tunable properties
    properties (Nontunable)
        FrameLength = 260;
        NumOfAntenna = 4;
    end

    % Discrete state properties
    properties (DiscreteState)

    end

    % Pre-computed constants or internal states
    properties (Access = private)

    end

    methods
        % Constructor
        function obj = CustomQRMode(varargin)
            % Support name-value pair arguments when constructing object
            setProperties(obj,nargin,varargin{:})
        end

         function set.FrameLength(obj,val)
            % validateattributes(val,{'double','single'},{'real','nonnegative','finite'},'Frame length');
            obj.FrameLength = val;
         end

          function set.NumOfAntenna(obj,val)
            % validateattributes(val,{'double','single'},{'real','nonnegative','finite'},'Number of antenna');
            obj.NumOfAntenna = val;
        end
    end

    methods (Access = protected)
        %% Common functions
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
        end

        function [R , p] = stepImpl(~,A)
            [m, n] = size(A);
            p = int32(1:n);
            H = @(u,x) x -  u*(u'*x);

            for k = 1:min(m, n)

                % Find column with maximum norm for column pivoting
                [~, idx] = max(vecnorm(A(k:end, k:end), 2, 1));
                idx = int32(idx + k - 1); % Ensure integer type for HDL code generation

                if idx ~= k
                    % Swap columns in A and p
                    A(:, [k, idx]) = A(:, [idx, k]);
                    p([k, idx]) = p([idx, k]);
                end

                % Apply Householder transformation
                x = A(k:m, k);
                h = housegen(x);
                A(k:m, k:n) = H(h ,  A(k:m, k:n));

            end

            % Extract R from the upper triangular part of A
            % R = triu(A(1:n, :));
            R = A(1:n, :);
        end

        function resetImpl(obj)
            % Initialize / reset internal or discrete properties
        end

        
        function u = housegen(x)
            % u = house_gen(x)
            % Generate Householder reflection.
            % u = house_gen(x) returns u with norm(u) = sqrt(2), and
            % H(u,x) = x - u*(u'*x) = -+ norm(x)*e_1.

            % Modify the sign function so that sign(0) = 1.
            sig = @(u) sign(u) + (u==0);

            nu = norm(x);
            if nu ~= 0
                u = x/nu;
                u(1) = u(1) + sig(u(1));
                u = u/sqrt(abs(u(1)));

            else
                u = x;
                u(1) = sqrt(2);
            end
        end


        %% Backup/restore functions
        function s = saveObjectImpl(obj)
            % Set properties in structure s to values in object obj

            % Set public properties and states
            s = saveObjectImpl@matlab.System(obj);

            % Set private and protected properties
            %s.myproperty = obj.myproperty;
        end

        function loadObjectImpl(obj,s,wasLocked)
            % Set properties in object obj to values in structure s

            % Set private and protected properties
            % obj.myproperty = s.myproperty; 

            % Set public properties and states
            loadObjectImpl@matlab.System(obj,s,wasLocked);
        end

        %% Simulink functions
        function ds = getDiscreteStateImpl(obj)
            % Return structure of properties with DiscreteState attribute
            ds = struct([]);
        end

        function flag = isInputSizeMutableImpl(obj,index)
            % Return false if input size cannot change
            % between calls to the System object
            flag = false;
        end

        function [out1 , out2] = getOutputSizeImpl(obj)
            % Return size for each output port
            out2 = [1 obj.NumOfAntenna];
            out1 = [obj.NumOfAntenna obj.NumOfAntenna];


            % Example: inherit size from first input port
            % out = propagatedInputSize(obj,1);
        end

        function [out,out2] = getOutputDataTypeImpl(obj)
            % Return data type for each output port
            out = "int32";
            out2 = "int32";

            % Example: inherit data type from first input port
            % out = propagatedInputDataType(obj,1);
        end

        function [out1,out2] = isOutputComplexImpl(obj)
            % Return true for each output port with complex data
            out1 = true;
            out2 = true;

            % Example: inherit complexity from first input port
            % out = propagatedInputComplexity(obj,1);
        end

        function [out1,out2] = isOutputFixedSizeImpl(obj)
            % Return true for each output port with fixed size
            out1 = true;
            out2 = true;

            % Example: inherit fixed-size status from first input port
            % out = propagatedInputFixedSize(obj,1);
        end

        function icon = getIconImpl(obj)
            % Define icon for System block
            icon = mfilename("class"); % Use class name
            % icon = "My System"; % Example: text icon
            % icon = ["My","System"]; % Example: multi-line text icon
            % icon = matlab.system.display.Icon("myicon.jpg"); % Example: image file icon
        end
    end

    methods (Static, Access = protected)
        %% Simulink customization functions
        function header = getHeaderImpl
            % Define header panel for System block dialog
            header = matlab.system.display.Header(mfilename("class"));
        end

        function group = getPropertyGroupsImpl
            % Define property section(s) for System block dialog
            group = matlab.system.display.Section(mfilename("class"));
        end
    end
end
