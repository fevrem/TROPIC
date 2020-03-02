function v = CAT_INDEPENDENT_PARAMETER()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = casadiMEX(0, 131);
  end
  v = vInitialized;
end
