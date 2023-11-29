#!/usr/bin/env python3

import os

def delete_line(text, index):
    lines = text.splitlines()
    del lines[index]
    return "\n".join(lines)

# Get the template file path
template_file_path = "./template.md"

# Check if the template file exists
if not os.path.exists(template_file_path):
    print("Error: Template file not found:", template_file_path)
    exit(1)

# Read the template file
with open(template_file_path, 'r') as template_file:
    template_content = template_file.read()

date = input("Enter the date of the mission (dd-mm-yyyy): ")
summary = input("Enter the summary of the mission: ")
footage_url = input("Enter the footage URL: ")
log_ist_fc = input("Enter the M690B IST flight computer log URL (empty if not existant): ")
log_ist_oc = input("Enter the M690B IST onboard computer log URL (empty if not existant): ")
log_nova_fc = input("Enter the M690B NOVA flight computer log URL (empty if not existant): ")
log_nova_oc = input("Enter the M690B NOVA onboard computer log URL (empty if not existant): ")
checklist_id = input("Enter just the checklist url ID (i.e.: https://docs.google.com/spreadsheets/d/ID/edit#gid=0): ")

if len(log_ist_fc) == 0:
    template_content = "\n".join(x for x in template_content.splitlines() if "log_ist_fc" not in x)
if len(log_ist_oc) == 0:
    template_content = "\n".join(x for x in template_content.splitlines() if "log_ist_oc" not in x)
if len(log_nova_fc) == 0:
    template_content = "\n".join(x for x in template_content.splitlines() if "log_nova_fc" not in x)
if len(log_nova_oc) == 0:
    template_content = "\n".join(x for x in template_content.splitlines() if "log_nova_oc" not in x)

if len(log_ist_fc) == 0 and len(log_ist_oc) == 0:
    template_content = "\n".join(x for x in template_content.splitlines() if "M690B IST" not in x)

if len(log_nova_fc) == 0 and len(log_nova_oc) == 0:
    template_content = "\n".join(x for x in template_content.splitlines() if "M690B NOVA" not in x)

template_content = template_content.replace("{{ date }}", date)
template_content = template_content.replace("{{ summary }}", summary)
template_content = template_content.replace("{{ footage_url }}", footage_url)
template_content = template_content.replace("{{ log_ist_fc }}", log_ist_fc)
template_content = template_content.replace("{{ log_ist_oc }}", log_ist_oc)
template_content = template_content.replace("{{ log_nova_fc }}", log_nova_fc)
template_content = template_content.replace("{{ log_nova_oc }}", log_nova_oc)
template_content = template_content.replace("{{ checklist_id }}", checklist_id)

# Get the output file path
output_file_path = "./" + date + ".md"

output_file_path_main = "../docs/Mission Logs/" + date + ".md"

if input("Do you want to place the generated file directly in the Mission Logs folder (y/n): ") == 'y':
    output_file_path = output_file_path_main

# Write the output content to the output file
with open(output_file_path, 'w') as output_file:
    output_file.write(template_content)

print("Output file generated:", output_file_path)

print("- " + date + ": 'Mission Logs/" + date + ".md'")