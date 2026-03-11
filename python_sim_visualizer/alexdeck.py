import random

# ====== YOUR DECK ======
your_deck = (
    ["Scroll Thief"] * 4 +
    ["Stealer of Secrets"] * 2 +
    ["Young Pyromancer"] * 2 +
    ["Archaeomancer"] * 3 +
    ["Jeskai Shrinekeeper"] * 2 +
    ["Narset, Jeskai Waymaster"] +
    ["Omenspeaker"] +
    ["Whirlwing Stormbrood"] +

    ["Leap"] * 3 +
    ["Taigam's Strike"] * 2 +
    ["Riverwheel Sweep"] * 2 +

    ["Shock"] * 2 +
    ["Scouring Sands"] * 2 +
    ["Flames of the Firebrand"] +
    ["Searing Spear"] * 2 +
    ["Glacial Dragonhunt"] +

    ["Anticipate"] * 3 +
    ["Opportunity"] +
    ["Rain of Revelation"] +

    ["Curiosity"] +
    ["Wingspan Stride"] * 2 +

    ["Island"] * 11 +
    ["Mountain"] * 5 +
    ["Dual Land"] * 5
)

# ====== MY DECK (Mono-Green Devotion) ======
my_deck = (
    ["Cauldron Familiar"] * 4 +
    ["Bloodtithe Harvester"] * 4 +
    ["Mayhem Devil"] * 4 +
    ["Woe Strider"] * 4 +
    ["Kroxa, Titan of Death’s Hunger"] * 2 +
    ["Unlucky Witness"] * 4 +

    ["Witch’s Oven"] * 4 +

    ["Fable of the Mirror-Breaker"] * 2 +

    ["Claim the Firstborn"] * 4 +
    ["Fatal Push"] * 2 +
    ["Kolaghan’s Command"] * 2 +

    ["Swamp"] * 10 +
    ["Mountain"] * 6 +
    ["RB Dual Land lifegain"] * 8
)


# Shuffle decks
random.shuffle(your_deck)
random.shuffle(my_deck)

print("Decks shuffled.")
print("Type 1 to draw from YOUR deck")
print("Type 2 to draw from MY deck")
print("Type q to quit\n")

while True:
    cmd = input("Draw (1 / 2 / q): ").strip()

    if cmd == "1":
        if your_deck:
            card = your_deck.pop(0)
            print(f"You drew: {card}  |  Cards left: {len(your_deck)}")
        else:
            print("Your deck is empty.")

    elif cmd == "2":
        if my_deck:
            card = my_deck.pop(0)
            print(f"I drew: {card}  |  Cards left: {len(my_deck)}")
        else:
            print("My deck is empty.")

    elif cmd.lower() == "q":
        print("Game ended.")
        break

    else:
        print("Invalid input. Use 1, 2, or q.")
