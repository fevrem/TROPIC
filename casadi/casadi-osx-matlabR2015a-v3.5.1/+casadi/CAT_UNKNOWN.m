function v = CAT_UNKNOWN()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = casadiMEX(0, 125);
  end
  v = vInitialized;
end
