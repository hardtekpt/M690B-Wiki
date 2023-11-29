#!/usr/bin/env python3

import yaml
import getpass
import git

def clone_repository(repo_url, folder_path):
    git.Repo.clone_from(repo_url, folder_path)

def check_git_repo(folder_path):
    try:
        git.Repo(folder_path, search_parent_directories=True)
        return True
    except git.InvalidGitRepositoryError:
        return False

def find_object(data, username):
    for obj in data:
        if obj['name'] == username:
            return obj
    return None

def parse_yaml(filename):
    with open(filename, 'r') as file:
        data = yaml.safe_load(file)
    return data

def pull_changes(folder_path):
    repo = git.Repo(folder_path, search_parent_directories=True)
    repo.remotes.origin.pull()

if __name__ == '__main__':

    # read packages_per_drone.yaml
    filename = 'packages_per_drone.yaml'
    data = parse_yaml(filename)

    # check which drone 
    username = getpass.getuser()
    drone = ""

    if username == "helios":
        drone = "m690b_ist"
    if username == "icarus":
        drone = "m690b_nova"

    print("Drone: " + drone)
    obj = find_object(data, drone)

    # check which packages
    for pkg in obj['packages']:

        print("Package name: " + pkg['name'])

        install = False
        update = False

        # check if packages exist in location
        folder_path = obj['base_path'] + pkg['name']
        #folder_path = "../test/" + pkg['name']
        try:
            if check_git_repo(folder_path):
                print("Git repository exists in folder:", folder_path)
                update = True
            else:
                print("Git repository does not exist in folder:", folder_path)
                install = True
        except:
            print("Path does not exist: " + folder_path)
            install = True

        # if true check if they need to be updated
        if update:
            pull_changes(folder_path)
            print("Updated git repository: " + pkg['name'])

        # if false install the packages
        if install:
            clone_repository(pkg['repo'], folder_path)
            print("Cloned git repository: " + pkg['name'])