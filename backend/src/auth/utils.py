import secrets
import string


def generate_secret_key(length: int = 64) -> str:
    """
    Generate a secure random secret key for JWT signing.

    Args:
        length: Length of the secret key (default: 64 characters)

    Returns:
        A secure random string suitable for JWT signing
    """
    alphabet = string.ascii_letters + string.digits + string.punctuation
    return ''.join(secrets.choice(alphabet) for _ in range(length))


def validate_secret_key(secret: str) -> bool:
    """
    Validate that a secret key meets minimum security requirements.

    Args:
        secret: The secret key to validate

    Returns:
        True if the secret meets requirements, False otherwise
    """
    if not secret:
        return False

    # Check minimum length
    if len(secret) < 32:
        return False

    # Check for sufficient complexity (at least one of each: upper, lower, digit, special)
    has_upper = any(c.isupper() for c in secret)
    has_lower = any(c.islower() for c in secret)
    has_digit = any(c.isdigit() for c in secret)
    has_special = any(c in string.punctuation for c in secret)

    return has_upper and has_lower and has_digit and has_special


if __name__ == "__main__":
    # Generate and print a new secret key
    new_secret = generate_secret_key()
    print(f"New secret key: {new_secret}")
    print(f"Key is valid: {validate_secret_key(new_secret)}")