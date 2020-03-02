classdef  FileSerializer < casadi.SerializerBase
    %FILESERIALIZER C++ includes: serializer.hpp 
    %
    %
    %
  methods
    function self = FileSerializer(varargin)
    %FILESERIALIZER Advanced serialization of CasADi objects.
    %
    %  new_obj = FILESERIALIZER()
    %
    %
    %StringSerializer, FileDeserializer
    %
    %
    %
      self@casadi.SerializerBase(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = casadiMEX(1120, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
        if self.swigPtr
          casadiMEX(1121, self);
          self.SwigClear();
        end
    end
  end
  methods(Static)
  end
end
