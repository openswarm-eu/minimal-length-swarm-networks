-- Local Modular

-- Synchronize local G
Gloc1 = sync(G1,G2)
Gloc2 = sync(G2,G3,G4)
Gloc3 = sync(G5,G6,G8,G9)
Gloc4 = sync(G8,G7_C)
Gloc5 = sync(G6,G7_F)
Gloc6 = sync(G5,G6,G8,G10,G15)
Gloc7 = sync(G5,G6,G13,G14)
Gloc8 = sync(G11,G12)

-- Synchronize local K
Kloc1 = sync(Gloc1,E1)
Kloc2 = sync(Gloc2,E2)
Kloc3 = sync(Gloc3,E3)
Kloc4 = sync(Gloc4,E4)
Kloc5 = sync(Gloc5,E5)
Kloc6 = sync(Gloc6,E6)
Kloc7 = sync(Gloc7,E7)
Kloc8 = sync(Gloc8,E8)

-- Create local supervisors
Sloc1 = supc(Gloc1, Kloc1)
Sloc2 = supc(Gloc2, Kloc2)
Sloc3 = supc(Gloc3, Kloc3)
Sloc4 = supc(Gloc4, Kloc4)
Sloc5 = supc(Gloc5, Kloc5)
Sloc6 = supc(Gloc6, Kloc6)
Sloc7 = supc(Gloc7, Kloc7)
Sloc8 = supc(Gloc8, Kloc8)

print("----------")
print(infom(Kloc1, Kloc2, Kloc3, Kloc4, Kloc5, Kloc6, Kloc7, Kloc8))
print(infom(Sloc1, Sloc2, Sloc3, Sloc4, Sloc5, Sloc6, Sloc7, Sloc8))

Kloc1 = minimize(Kloc1)
Kloc2 = minimize(Kloc2)
Kloc3 = minimize(Kloc3)
Kloc4 = minimize(Kloc4)
Kloc5 = minimize(Kloc5)
Kloc6 = minimize(Kloc6)
Kloc7 = minimize(Kloc7)
Kloc8 = minimize(Kloc8)

Sloc1 = minimize(Sloc1)
Sloc2 = minimize(Sloc2)
Sloc3 = minimize(Sloc3)
Sloc4 = minimize(Sloc4)
Sloc5 = minimize(Sloc5)
Sloc6 = minimize(Sloc6)
Sloc7 = minimize(Sloc7)
Sloc8 = minimize(Sloc8)

print("----------")
print(infom(Kloc1, Kloc2, Kloc3, Kloc4, Kloc5, Kloc6, Kloc7, Kloc8))
print(infom(Sloc1, Sloc2, Sloc3, Sloc4, Sloc5, Sloc6, Sloc7, Sloc8))

-- Add to Nadzoru
export( Sloc1 )
export( Sloc2 )
export( Sloc3 )
export( Sloc4 )
export( Sloc5 )
export( Sloc6 )
export( Sloc7 )
export( Sloc8 )
