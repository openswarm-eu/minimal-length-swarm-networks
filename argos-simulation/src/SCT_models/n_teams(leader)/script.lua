-- Local Modular

-- Synchronize local G
Gloc1 = sync(G1,G2)
Gloc2 = sync(G1,G2)

-- Synchronize local K
Kloc1 = sync(Gloc1,E1)
Kloc2 = sync(Gloc2,E2)

-- Create local supervisors
Sloc1 = supc(Gloc1, Kloc1)
Sloc2 = supc(Gloc2, Kloc2)

print("----------")
print(infom(Kloc1, Kloc2))
print(infom(Sloc1, Sloc2))

Kloc1 = minimize(Kloc1)
Kloc2 = minimize(Kloc2)

Sloc1 = minimize(Sloc1)
Sloc2 = minimize(Sloc2)

print("----------")
print(infom(Kloc1, Kloc2))
print(infom(Sloc1, Sloc2))

-- Add to Nadzoru
export( Sloc1 )
export( Sloc2 )
