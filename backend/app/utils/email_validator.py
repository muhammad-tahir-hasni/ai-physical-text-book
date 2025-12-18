"""
Email validation utilities for verifying real email addresses.
"""

import dns.resolver
import re
from typing import Tuple


# List of disposable/temporary email domains to block
DISPOSABLE_EMAIL_DOMAINS = {
    'tempmail.com', 'guerrillamail.com', 'mailinator.com', '10minutemail.com',
    'throwaway.email', 'maildrop.cc', 'temp-mail.org', 'getnada.com',
    'trashmail.com', 'yopmail.com', 'fakeinbox.com', 'sharklasers.com'
}


def validate_email_domain(email: str) -> Tuple[bool, str]:
    """
    Validate that an email domain is real and not disposable.

    Args:
        email: Email address to validate

    Returns:
        Tuple of (is_valid, error_message)
        - (True, "") if valid
        - (False, error_message) if invalid
    """
    # Extract domain from email
    try:
        domain = email.split('@')[1].lower()
    except IndexError:
        return False, "Invalid email format"

    # Check if domain is in disposable list
    if domain in DISPOSABLE_EMAIL_DOMAINS:
        return False, "Temporary or disposable email addresses are not allowed"

    # Check if domain has valid MX records (indicates real email server)
    try:
        mx_records = dns.resolver.resolve(domain, 'MX')
        if not mx_records:
            return False, "Email domain does not have valid mail servers"
        return True, ""
    except dns.resolver.NXDOMAIN:
        return False, "Email domain does not exist"
    except dns.resolver.NoAnswer:
        return False, "Email domain does not have mail servers configured"
    except dns.resolver.Timeout:
        # Don't block on timeout - could be network issue
        return True, ""
    except Exception as e:
        # Don't block on other DNS errors - could be temporary
        print(f"⚠️ Email validation warning for {email}: {str(e)}")
        return True, ""


def is_valid_professional_email(email: str) -> Tuple[bool, str]:
    """
    Check if email is from a professional/official domain.
    For production use, you might want to require institutional emails.

    Args:
        email: Email address to check

    Returns:
        Tuple of (is_valid, error_message)
    """
    domain = email.split('@')[1].lower()

    # Allow common professional email providers
    professional_domains = {
        'gmail.com', 'yahoo.com', 'outlook.com', 'hotmail.com',
        'icloud.com', 'protonmail.com', 'aol.com'
    }

    # Check if it's a known professional domain or institutional (.edu, .org, company domains)
    if domain in professional_domains:
        return True, ""

    # Allow .edu, .org, and other institutional domains
    if domain.endswith(('.edu', '.ac.uk', '.edu.au', '.org')):
        return True, ""

    # For other domains, just verify they're real via MX records
    return validate_email_domain(email)
