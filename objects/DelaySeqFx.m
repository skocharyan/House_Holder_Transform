classdef (StrictDefaults) DelaySeqFx < matlab.System
    % untitled4 Add summary here
    %
    % NOTE: When renaming the class name untitled4, the file name
    % and constructor name must be updated to use the class name.
    %
    % This template includes most, but not all, possible properties, attributes,
    % and methods that you can implement for a System object in Simulink.

    % Public, tunable properties
    properties

    end

    % Public, non-tunable properties
    properties (Nontunable)
        % NumOfAntenna = 4;
        % FrameLength = 260;
    end

    % Discrete state properties
    properties (DiscreteState)

    end

    % Pre-computed constants or internal states
    properties (Access = private)

    end

    methods
        % Constructor
        function obj = DelaySeqFx(varargin)
            % Support name-value pair arguments when constructing object
            setProperties(obj,nargin,varargin{:})
        end



        function y=delaySeqFx(x,dtIn,fs)
            %delayseq Delay or advance time sequence
            %   Y = delayseq(X, DELAY) returns the delayed or advanced sequence Y by
            %   applying DELAY to the input sequence X. DELAY (in samples) can be
            %   integer or non-integer values. When it is negative, the sequence X is
            %   advanced. X can be a vector or a matrix. DELAY is a scalar or a vector.
            %
            %   When X is a column vector, X is delayed by each element of DELAY and
            %   the resulting sequence will be stored in corresponding column of Y.
            %
            %   When X has multiple columns, each column is delayed by corresponding
            %   element of DELAY. If DELAY is a scalar, it will be applied to each
            %   column of X.
            %
            %   The output sequence Y always has the same length as input with
            %   appropriate truncations or zero padding.
            %
            %   Y = delayseq(X, DELAY, Fs) specifies DELAY in seconds. Fs is the
            %   sampling frequency (in Hz).
            %
            %   This function supports single and double precision for input data and
            %   arguments. If the input data X is single precision, the output data
            %   is single precision. If the input data X is double precision, the
            %   output data is double precision. The precision of the output is



            % % validateattributes(fs, {'double','single'},...
            % %     {'scalar', 'real', 'positive', 'finite', 'nonnan'},...
            % %     'delayseq', 'Fs');
            % convert dt from time to samples
            dtIn = dtIn*fs;



            bSingleVecX = false;
            if (length(dtIn) == 1)    % scalar expansion of dt
                dt = dtIn*ones(size(x,2),1);
            else
                dt = dtIn;
            end


            cond = size(x,2)~=1 && (size(x,2)~=length(dt));
            if cond
                coder.internal.errorIf(cond,'phased:delayseq:MismatchedDelay');
            end


            % initialization
            inputLength = size(x,1);
            delayInt = round(dt);    % Integer delays in samples
            delayFrac = dt - delayInt;
            maxLength = inputLength+max(0, max(delayInt)); % maximum sequence length
            % Define upperbound
            if maxLength > 2*inputLength
                maxLengthLimit = 2*inputLength;
            else
                maxLengthLimit = maxLength;
            end
            nfftLimit = 2^ceil(log2((2*inputLength)));
            if isreal(x)
                output = zeros(maxLengthLimit,length(dt),class(x));
            else
                output = complex(zeros(maxLengthLimit,length(dt),class(x)));
            end

            % Perform delay operation
            for colI=1:length(dt)

                endIdx = inputLength+delayInt(colI);
                % no operation if delayed or advanced out of scope
                if (endIdx <= 0) || (endIdx > 2*inputLength)
                    continue;
                end
                tmpx = x(:,colI);

                if delayFrac(colI)
                    nfft = 2^ceil(log2(inputLength + max(0, delayInt(colI))));

                    assert(nfft <= nfftLimit);
                    binStart = floor(nfft/2);
                    % Notice the FFT bins must belong to [-pi, pi].
                    fftBin = 2*pi*ifftshift(((0:nfft-1)-binStart).')/nfft;
                    if isscalar(tmpx) && nfft==1

                        tmpxd = tmpx;
                    else

                        tmpxd = fft(tmpx,nfft);
                        tmpxd = ifft(tmpxd(:).*exp(-1i*dt(colI)*fftBin));
                    end

                    if delayInt(colI) >= 0
                        orgStart = delayInt(colI) + 1;
                        newStart = delayInt(colI) + 1;
                    else
                        orgStart = 1;
                        newStart = 1;
                    end
                    orgEnd = endIdx;
                    if isreal(x)
                        output(newStart:endIdx,colI) = real(tmpxd(orgStart:orgEnd));
                    else
                        output(newStart:endIdx,colI) = tmpxd(orgStart:orgEnd);
                    end
                else
                    % Integer sampling shifting
                    if delayInt(colI) >= 0
                        orgStart = 1;
                        newStart = delayInt(colI) + 1;
                    else
                        orgStart = 1-delayInt(colI);
                        newStart = 1;
                    end
                    orgEnd = inputLength;
                    output(newStart:endIdx,colI) = tmpx(orgStart:orgEnd);
                end



            end

            % output has the same length as input
            y = output(1:inputLength, :);

        end


    end

    methods (Access = protected)
        %% Common functions
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
        end

        function y = stepImpl(obj,x,dtIn,fs)
            dtIn = dtIn*fs;
            bSingleVecX = false;
            if (length(dtIn) == 1)    % scalar expansion of dt
                dt = dtIn*ones(size(x,2),1);
            else
                dt = dtIn;
            end
            cond = size(x,2)~=1 && (size(x,2)~=length(dt));
            if cond
                coder.internal.errorIf(cond,'phased:delayseq:MismatchedDelay');
            end
            % initialization
            inputLength = size(x,1);
            delayInt = round(dt);    % Integer delays in samples
            delayFrac = dt - delayInt;
            maxLength = inputLength+max(0, max(delayInt)); % maximum sequence length
            % Define upperbound
            if maxLength > 2*inputLength
                maxLengthLimit = 2*inputLength;
            else
                maxLengthLimit = maxLength;
            end
            nfftLimit = 2^ceil(log2((2*inputLength)));
            if isreal(x)
                output = zeros(maxLengthLimit,length(dt),class(x));
            else
                output = complex(zeros(maxLengthLimit,length(dt),class(x)));
            end

            % Perform delay operation
            for colI=1:length(dt)

                endIdx = inputLength+delayInt(colI);
                % no operation if delayed or advanced out of scope
                if (endIdx <= 0) || (endIdx > 2*inputLength)
                    continue;
                end
                tmpx = x(:,colI);

                if delayFrac(colI)
                    nfft = 2^ceil(log2(inputLength + max(0, delayInt(colI))));

                    assert(nfft <= nfftLimit);
                    binStart = floor(nfft/2);
                    % Notice the FFT bins must belong to [-pi, pi].
                    fftBin = 2*pi*ifftshift(((0:nfft-1)-binStart).')/nfft;
                    if isscalar(tmpx) && nfft==1

                        tmpxd = tmpx;
                    else

                        tmpxd = fft(tmpx,nfft);
                        tmpxd = ifft(tmpxd(:).*exp(-1i*dt(colI)*fftBin));
                    end

                    if delayInt(colI) >= 0
                        orgStart = delayInt(colI) + 1;
                        newStart = delayInt(colI) + 1;
                    else
                        orgStart = 1;
                        newStart = 1;
                    end
                    orgEnd = endIdx;
                    if isreal(x)
                        output(newStart:endIdx,colI) = real(tmpxd(orgStart:orgEnd));
                    else
                        output(newStart:endIdx,colI) = tmpxd(orgStart:orgEnd);
                    end
                else
                    % Integer sampling shifting
                    if delayInt(colI) >= 0
                        orgStart = 1;
                        newStart = delayInt(colI) + 1;
                    else
                        orgStart = 1-delayInt(colI);
                        newStart = 1;
                    end
                    orgEnd = inputLength;
                    output(newStart:endIdx,colI) = tmpx(orgStart:orgEnd);
                end



            end

            % output has the same length as input
            y = output(1:inputLength, :);
        end

        function resetImpl(obj)
            % Initialize / reset internal or discrete properties
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

        function out  = getOutputSizeImpl(obj)
            out = propagatedInputSize(obj,1);
        end

        function out = getOutputDataTypeImpl(obj)
            out = propagatedInputDataType(obj,1);
        end

        function out = isOutputComplexImpl(obj)
            out = propagatedInputComplexity(obj,1);
        end

        function out = isOutputFixedSizeImpl(obj)
            out = propagatedInputFixedSize(obj,1);
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
