import random

deck = (
    ["Augur of Bolas"] * 3 +
    ["Baleful Strix"] * 2 +
    ["Serrated Scorpion"] * 3 +
    ["Gurmag Angler"] * 2 +
    ["Archaeomancer"] * 2 +
    ["Mist Raven"] * 2 +
    ["Phyrexian Rager"] * 2 +

    ["Doom Blade"] * 2 +
    ["Disfigure"] * 2 +
    ["Essence Scatter"] * 2 +
    ["Negate"] * 2 +
    ["Agony Warp"] * 2 +

    ["Preordain"] * 3 +
    ["Think Twice"] * 2 +
    ["Read the Bones"] * 1 +
    ["Opportunity"] * 1 +
    ["Unearth"] * 2 +
    ["Into the Roil"] * 2 +

    ["Dead Weight"] * 2 +

    ["Island"] * 9 +
    ["Swamp"] * 6 +
    ["UB Lifegain Tapland"] * 4 +
    ["Evolving Wilds"] * 2
)

random.shuffle(deck)

draw_count = 0

while deck:
    input("Press Enter to draw a card...")
    card = deck.pop()
    draw_count += 1
    print(f"Draw {draw_count}: {card}")
    print(f"Cards remaining: {len(deck)}\n")

print("Deck is empty.")
