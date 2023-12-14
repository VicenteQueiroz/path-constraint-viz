class Node:
    def __init__(self, info):
        self.info = info
        self.left = None
        self.right = None
        self.level = None

    def __str__(self):
        return str(self.info)


class BinarySearchTree:
    def __init__(self):
        self.root = None

    def create(self, val):
        if self.root == None:
            self.root = Node(val)
        else:
            current = self.root

            while True:
                if val < current.info:
                    if current.left:
                        current = current.left
                    else:
                        current.left = Node(val)
                        break
                elif val > current.info:
                    if current.right:
                        current = current.right
                    else:
                        current.right = Node(val)
                        break
                else:
                    break


def height(root):
    aux = 0
    if root.left:
        aux = height(root.left) + 1
    if root.right:
        aux = height(root.right) + 1
    return aux


def levelOrder(root):
    res = []
    if root:
        res = levelOrder(root.left)
        res = res + levelOrder(root.right)
        res.append(root.info)
    return res


tree = BinarySearchTree()

t = 7
arr = list(map(int, "3 2 5 1 4 6 7".split()))

for i in range(t):
    tree.create(arr[i])

print(height(tree.root))

print(levelOrder(tree.root))