classdef HKHoverLogs
 % Small immutable adapter for existing .get-based log collectors.
 properties (SetAccess=private)
  Data
 end
 methods
  function obj=HKHoverLogs(data),obj.Data=data;end
  function value=get(obj,name)
   assert(isfield(obj.Data,name),'flappy:vectorLog','Missing log: %s',name);
   value=obj.Data.(name);
  end
 end
end
