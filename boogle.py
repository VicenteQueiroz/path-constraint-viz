dictionary = ["GEEKS", "FOR", "QUIZ", "GO"]

boggle = [["G", "I", "Z"], ["U", "E", "K"], ["Q", "S", "E"]]

finalResult = []


def findWordAdjacent(word, i, j, index):
    # check first character
    if word[index] == boggle[i][j]:
        # check if found the word!
        if (len(word) - 1) == index:
            print("found word: ", word)
            finalResult.append(word)
            return True

        if i != 0:
            # check up:
            findWordAdjacent(word, i - 1, j, index + 1)
        if i != 0 and (j != len(boggle[i]) - 1):
            # check up right:
            findWordAdjacent(word, i - 1, j + 1, index + 1)
        if j != len(boggle[i]) - 1:
            # check right:
            findWordAdjacent(word, i, j + 1, index + 1)
        if j != len(boggle[i]) - 1 and (i != len(boggle) - 1):
            # check down right:
            findWordAdjacent(word, i + 1, j + 1, index + 1)
        if i != len(boggle) - 1:
            # check down:
            findWordAdjacent(word, i + 1, j, index + 1)
        if i != len(boggle) - 1 and (j != 0):
            # check down left:
            findWordAdjacent(word, i + 1, j - 1, index + 1)
        if j != 0:
            # check left:
            findWordAdjacent(word, i, j - 1, index + 1)
        if (j != 0) and (i != 0):
            # check up left:
            findWordAdjacent(word, i - 1, j - 1, index + 1)

    else:
        return False


def solveBoggle(dict, boggle):
    for i in range(len(boggle)):
        for j in range(len(boggle[i])):
            for word in dict:
                # Check character in boggle, if matches check the adjacent, if not return
                findWordAdjacent(word, i, j, 0)


solveBoggle(dictionary, boggle)

print("______________________________________\nBoggle result!!!\n")
print(finalResult)
print("\n____________________________________")
