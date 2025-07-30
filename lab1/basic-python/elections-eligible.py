class Election:
    def __init__(self, name, age, location, gender):
        self.name = name
        self.age = age 
        self.location = location
        self.gender = gender 

    def check_eligible_or_not(self):
        if self.age >= 18:
            print(f"{self.name} from {self.location} is eligible to vote.")
        else:
            print(f"{self.name} from {self.location} is not eligible to vote.")

name = input("please enter your name: ")
birthday = int(input("please enter your age: "))
location = input("enter your location: ")
gender = input("please enter your gender: ")

friend = Election(name, age, location, gender)
friend.check_eligible_or_not()
