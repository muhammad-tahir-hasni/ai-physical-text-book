"""UserProfile model for storing user background questionnaire data."""

from sqlalchemy import Column, Integer, String, ForeignKey
from sqlalchemy.orm import relationship
from app.core.database import Base


class UserProfile(Base):
    """User profile model for background questionnaire data."""

    __tablename__ = "user_profiles"

    user_id = Column(Integer, ForeignKey("users.id", ondelete="CASCADE"), primary_key=True)
    software_experience = Column(String(20), nullable=True)  # beginner/intermediate/advanced
    python_familiarity = Column(String(20), nullable=True)  # none/basic/intermediate/advanced
    robotics_experience = Column(String(20), nullable=True)  # none/hobbyist/academic/professional
    hardware_background = Column(String(20), nullable=True)  # none/basic/intermediate/advanced
    learning_goals = Column(String(200), nullable=True)  # Free text
    preferred_complexity = Column(String(20), default="intermediate", nullable=False)  # beginner/intermediate/advanced

    # Relationship
    user = relationship("User", back_populates="profile")

    def __repr__(self):
        return f"<UserProfile(user_id={self.user_id}, preferred_complexity={self.preferred_complexity})>"
